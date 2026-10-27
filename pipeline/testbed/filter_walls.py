#!/usr/bin/env python3
"""
Remove large vertical planar surfaces (walls) from a LiDAR point cloud.

THE RETENTION DIAL
Once wall points are identified, they are not necessarily all discarded. Two
independent controls decide which survive, and they compose:

  ~keep_fraction    (0..1)   keep a random fraction of wall points
  ~keep_sector_deg  (0..360) keep wall points whose bearing falls inside a
                             sector of this width, centred on ~sector_center_deg

A wall point survives only if it passes BOTH. The endpoints are:

  keep_fraction=0.0, keep_sector_deg=0     all walls removed  (open terrain)
  keep_fraction=1.0, keep_sector_deg=360   all walls kept     (passthrough)


point LeGO-LOAM at the filtered topic, e.g. in run.launch:
    <remap from="/velodyne_points" to="/velodyne_points_filtered"/>
"""

import numpy as np
import rospy
import sensor_msgs.point_cloud2 as pc2
from sensor_msgs.msg import PointCloud2, Imu

# PointField datatype -> numpy scalar type
_PF_DTYPES = {
    1: np.int8,   2: np.uint8,  3: np.int16,   4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def cloud_dtype(msg):
    """Structured dtype matching the message's own point layout.

    itemsize is point_step, not the sum of the field sizes, so any padding the
    driver inserted is preserved and a record can be copied back out verbatim.
    """
    names, formats, offsets = [], [], []
    for f in msg.fields:
        if f.datatype not in _PF_DTYPES:
            continue
        names.append(f.name)
        formats.append(_PF_DTYPES[f.datatype])
        offsets.append(f.offset)
    return np.dtype({"names": names, "formats": formats,
                     "offsets": offsets, "itemsize": msg.point_step})


def as_record_array(msg):
    """View the raw buffer as a structured array -- no per-point Python objects."""
    n = msg.height * msg.width
    return np.frombuffer(msg.data, dtype=cloud_dtype(msg), count=n)


def make_cloud(msg, rec):
    """Rebuild a PointCloud2 from a subset of records, keeping every field.

    The record dtype came from the input, so the bytes drop straight back in
    and fields/point_step are unchanged. is_dense is set true because the
    non-finite points were removed upstream.
    """
    out = PointCloud2()
    out.header = msg.header
    out.height = 1
    out.width = len(rec)
    out.fields = msg.fields
    out.is_bigendian = msg.is_bigendian
    out.point_step = msg.point_step
    out.row_step = msg.point_step * len(rec)
    out.data = np.ascontiguousarray(rec).tobytes()
    out.is_dense = True
    return out


def column_span(pts, col_bin_deg):
    """Vertical extent of the inliers WITHIN narrow bearing columns.

    This is the test that separates a wall from a streak of ground, and it is
    the only one of the shape tests that works. A wall occupies several beams at
    a fixed bearing, so a column through it holds points at different heights. A
    streak of ground is a chord of ONE laser ring: every point in a column comes
    from the same beam, so the column has no vertical extent at all.
    """
    bearing = np.degrees(np.arctan2(pts[:, 1], pts[:, 0]))
    col = np.floor(bearing / col_bin_deg).astype(np.int64)
    order = np.argsort(col, kind="stable")
    c, z = col[order], pts[order, 2]
    starts = np.flatnonzero(np.r_[True, c[1:] != c[:-1]])
    spans = np.maximum.reduceat(z, starts) - np.minimum.reduceat(z, starts)
    return float(np.percentile(spans, 75))


def fit_vertical_planes(xyz, dist_thresh, min_inliers, max_planes,
                        iters, tilt_tol_deg, rng, score_subsample=6000,
                        min_extent=1.0, min_z_span=0.20, min_col_span=0.10,
                        col_bin_deg=0.5):
    """Iteratively find vertical planes.
    """
    n_pts = len(xyz)
    unassigned = np.ones(n_pts, dtype=bool)
    wall_mask = np.zeros(n_pts, dtype=bool)
    planes = []
    rejects = {"extent": 0, "z_span": 0, "col_span": 0}
    # A candidate pair whose XY separation is tiny gives a badly conditioned
    # normal, so require a minimum baseline.
    min_baseline = max(3.0 * dist_thresh, 0.20)
    cos_tol = np.cos(np.radians(90.0 - tilt_tol_deg))

    # Rejected candidates do not count against max_planes, so the loop needs a
    # separate bound on total work or a ground-heavy scan could spin.
    for _ in range(4 * max_planes + 8):
        if len(planes) >= max_planes:
            break
        idx = np.flatnonzero(unassigned)
        if len(idx) < min_inliers:
            break
        pts = xyz[idx]

        # Subsample used only for RANKING candidates (see PERFORMANCE above).
        if score_subsample > 0 and len(pts) > score_subsample:
            sel = rng.choice(len(pts), score_subsample, replace=False)
            score_pts = pts[sel]
        else:
            score_pts = pts
        scale = len(pts) / float(len(score_pts))

        i1 = rng.integers(0, len(pts), iters)
        i2 = rng.integers(0, len(pts), iters)
        p1, p2 = pts[i1], pts[i2]

        v = p2[:, :2] - p1[:, :2]
        L = np.linalg.norm(v, axis=1)
        ok = L > min_baseline
        if not ok.any():
            break
        v, p1, L = v[ok], p1[ok], L[ok]

        # normal perpendicular to v, in the XY plane -> vertical plane
        nrm = np.column_stack([-v[:, 1], v[:, 0]]) / L[:, None]
        d = np.einsum("ij,ij->i", nrm, p1[:, :2])

        # Rank candidates on the subsample: (n_sub, n_hyp) instead of
        # (n_pts, n_hyp). This is where the speedup comes from.
        resid_s = np.abs(score_pts[:, :2] @ nrm.T - d)   # (n_sub, n_hyp)
        counts = (resid_s < dist_thresh).sum(axis=0)

        best = int(np.argmax(counts))
        if counts[best] * scale < min_inliers:
            break

        # Assign inliers exactly, against the full remaining set, for the
        # winner only. One (n_pts,) computation rather than (n_pts, n_hyp).
        n_best, d_best = nrm[best], d[best]
        resid_best = np.abs(pts[:, :2] @ n_best - d_best)
        inl = resid_best < dist_thresh
        if inl.sum() < min_inliers:
            break

        w = pts[inl]

        # 1. Horizontal extent. Rejects a thin vertical column -- a post, or the
        #    edge of a rock -- which is vertical but not a wall.
        along = w[:, :2] @ np.array([-n_best[1], n_best[0]])
        extent = float(along.max() - along.min())
        if extent < min_extent:
            unassigned[idx[inl]] = False       # consumed, but not a wall
            rejects["extent"] += 1
            continue

        # 2. Overall height. Cheap, and catches the flattest fits.
        z_span = float(w[:, 2].max() - w[:, 2].min())
        if z_span < min_z_span:
            unassigned[idx[inl]] = False
            rejects["z_span"] += 1
            continue

        # 3. Height within a bearing column. This is the test that rejects a
        #    chord of a single ground ring, which tests 1 and 2 both pass.
        col_span = column_span(w, col_bin_deg)
        if col_span < min_col_span:
            unassigned[idx[inl]] = False
            rejects["col_span"] += 1
            continue

        planes.append((n_best, float(d_best), int(inl.sum()),
                       z_span, extent, col_span))
        wall_mask[idx[inl]] = True
        unassigned[idx[inl]] = False

    return wall_mask, planes, rejects


def sector_mask(xyz, center_deg, width_deg):
    """True for points whose sensor-frame bearing lies inside the sector.

    width >= 360 keeps everything; width <= 0 keeps nothing.
    """
    n = len(xyz)
    if width_deg >= 360.0:
        return np.ones(n, dtype=bool)
    if width_deg <= 0.0:
        return np.zeros(n, dtype=bool)
    bearing = np.degrees(np.arctan2(xyz[:, 1], xyz[:, 0]))
    delta = np.abs((bearing - center_deg + 180.0) % 360.0 - 180.0)
    return delta <= (width_deg / 2.0)


class WallFilter:
    def __init__(self):
        self.dist_thresh = rospy.get_param("~dist_thresh", 0.08)
        self.min_inliers = rospy.get_param("~min_inliers", 300)
        self.max_planes = rospy.get_param("~max_planes", 6)
        self.iters = rospy.get_param("~iters", 400)
        # Points used to RANK plane hypotheses. Inlier assignment always uses
        # the full cloud; this only bounds the scoring matrix. 0 disables.
        self.score_subsample = int(rospy.get_param("~score_subsample", 6000))
        self.tilt_tol = rospy.get_param("~tilt_tolerance_deg", 8.0)
        self.max_range = rospy.get_param("~max_range", 0.0)   # 0 = no limit
        # Near-field ground rings are the densest thing in the scan and the
        # easiest false positive, and the rover's own structure sits in there
        # too. Nothing we want to remove is within 2 m of the sensor -- the
        # retaining wall is >= 5 m away from anywhere inside the pit.
        self.min_range = rospy.get_param("~min_range", 2.0)

        # --- shape tests (see fit_vertical_planes) ---------------------------
        self.min_extent = float(rospy.get_param("~min_extent", 1.0))
        self.min_z_span = float(rospy.get_param("~min_z_span", 0.20))
        self.min_col_span = float(rospy.get_param("~min_col_span", 0.10))
        self.col_bin_deg = float(rospy.get_param("~col_bin_deg", 0.5))

        # --- retention dial -------------------------------------------------
        self.keep_fraction = float(rospy.get_param("~keep_fraction", 0.0))
        self.keep_sector = float(rospy.get_param("~keep_sector_deg", 0.0))
        self.sector_center = float(rospy.get_param("~sector_center_deg", 0.0))
        self.keep_fraction = min(max(self.keep_fraction, 0.0), 1.0)

        self.debug = rospy.get_param("~debug", False)
        self.imu_topic = rospy.get_param("~imu_topic", "")
        seed = int(rospy.get_param("~seed", -1))
        if seed < 0:
            seed = int(np.random.SeedSequence().entropy % (2 ** 31))
        self.seed = seed

        # Separate streams so that changing keep_fraction does not perturb
        # which planes RANSAC finds. Without this, two sweep levels would
        # differ in their plane detection as well as their retention, and the
        # manipulation would not be clean.
        ss = np.random.SeedSequence(self.seed).spawn(2)
        self.rng = np.random.default_rng(ss[0])          # RANSAC
        self.rng_keep = np.random.default_rng(ss[1])     # decimation

        self.R_level = None          # set from IMU if enabled
        self.n_scans = 0
        self.removed_frac = []
        self.wall_frac = []
        self.kept_wall_frac = []
        self.warned_no_ring = False
        self.rejects = {"extent": 0, "z_span": 0, "col_span": 0}
        self.n_planes = []

        self.pub = rospy.Publisher("~output", PointCloud2, queue_size=2)
        # Debug topics for RViz. ~removed carries the wall points the dial threw
        # away; ~walls carries every point classified as wall regardless of the
        # dial. Displaying ~output and ~removed together in different colours
        # shows exactly what the filter is doing at each level.
        self.publish_debug = rospy.get_param("~publish_debug", True)
        if self.publish_debug:
            self.pub_removed = rospy.Publisher("~removed", PointCloud2, queue_size=2)
            self.pub_walls = rospy.Publisher("~walls", PointCloud2, queue_size=2)
        rospy.Subscriber("~input", PointCloud2, self.cb, queue_size=2,
                         buff_size=2 ** 24)
        if self.imu_topic:
            rospy.Subscriber(self.imu_topic, Imu, self.imu_cb, queue_size=10)
        rospy.on_shutdown(self.report)

        rospy.loginfo(
            "filter_walls dial: keep_fraction=%.3f keep_sector_deg=%.1f "
            "(centre %.1f) seed=%d",
            self.keep_fraction, self.keep_sector, self.sector_center, self.seed)
        if self.keep_fraction >= 1.0 and self.keep_sector >= 360.0:
            rospy.logwarn("filter_walls: dial fully open -- walls kept intact. "
                          "This is the passthrough condition.")

    def imu_cb(self, msg):
        """Build a rotation that removes roll and pitch, so 'vertical' is
        world-vertical rather than sensor-vertical. Yaw is deliberately left
        alone -- rotating about z does not affect which planes are vertical."""
        q = msg.orientation
        x, y, z, w = q.x, q.y, q.z, q.w
        # gravity direction in sensor frame
        g = np.array([2 * (x * z - w * y),
                      2 * (y * z + w * x),
                      1 - 2 * (x * x + y * y)])
        g /= np.linalg.norm(g) + 1e-12
        target = np.array([0.0, 0.0, 1.0])
        v = np.cross(g, target)
        s, c = np.linalg.norm(v), float(np.dot(g, target))
        if s < 1e-8:
            self.R_level = np.eye(3)
            return
        K = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
        self.R_level = np.eye(3) + K + K @ K * ((1 - c) / s ** 2)

    def cb(self, msg):
        # Structured view over the raw buffer: no per-point Python objects, and
        # every field survives -- including `ring`, which the consumer needs
        # when useCloudRing = true.
        rec = as_record_array(msg)
        if "ring" not in rec.dtype.names and not self.warned_no_ring:
            rospy.logwarn("filter_walls: input has no 'ring' field. Fine if the "
                          "consumer runs useCloudRing = false, fatal if not.")
            self.warned_no_ring = True

        xyz_all = np.column_stack([rec["x"].astype(np.float64),
                                   rec["y"].astype(np.float64),
                                   rec["z"].astype(np.float64)])
        finite = np.isfinite(xyz_all).all(axis=1)      # replaces skip_nans
        rec = rec[finite]
        xyz = xyz_all[finite]

        if len(rec) < self.min_inliers:
            self.pub.publish(make_cloud(msg, rec))
            return

        work = xyz @ self.R_level.T if self.R_level is not None else xyz

        rng_xy = np.linalg.norm(work[:, :2], axis=1)
        valid = np.ones(len(work), dtype=bool)
        if self.max_range > 0:
            valid &= rng_xy < self.max_range
        if self.min_range > 0:
            valid &= rng_xy > self.min_range

        # 1. identify wall points
        wall = np.zeros(len(work), dtype=bool)
        sub, planes, rejects = fit_vertical_planes(
            work[valid], self.dist_thresh, self.min_inliers, self.max_planes,
            self.iters, self.tilt_tol, self.rng, self.score_subsample,
            self.min_extent, self.min_z_span, self.min_col_span,
            self.col_bin_deg)
        wall[np.flatnonzero(valid)] = sub
        for k, v in rejects.items():
            self.rejects[k] += v

        # 2. decide which wall points survive the dial
        n_wall = int(wall.sum())
        if n_wall == 0:
            keep = np.ones(len(work), dtype=bool)
            kept_wall = 0
        else:
            survive = sector_mask(work, self.sector_center, self.keep_sector)
            if self.keep_fraction <= 0.0:
                survive = np.zeros(len(work), dtype=bool)
            elif self.keep_fraction < 1.0:
                survive &= self.rng_keep.random(len(work)) < self.keep_fraction
            keep = ~wall | survive
            kept_wall = int((wall & keep).sum())

        out = rec[keep]
        if len(out) == 0:
            rospy.logwarn_throttle(
                5.0, "filter_walls: filtered scan is empty -- publishing the "
                     "original so the pipeline does not stall")
            self.pub.publish(make_cloud(msg, rec))
            return

        frac = 1.0 - len(out) / len(rec)
        self.n_scans += 1
        self.removed_frac.append(frac)
        self.wall_frac.append(n_wall / len(rec))
        self.kept_wall_frac.append(
            kept_wall / n_wall if n_wall else float("nan"))

        self.n_planes.append(len(planes))

        if self.debug and self.n_scans % 10 == 0:
            rospy.loginfo(
                f"scan {self.n_scans}: {len(rec)} -> {len(out)} pts "
                f"({100 * frac:.1f}% removed), {len(planes)} planes, "
                f"wall pts {n_wall}, retained {kept_wall} "
                f"({100 * kept_wall / n_wall if n_wall else 0.0:.1f}%)")
            for k, (nv, dv, cnt, zs, ext, cs) in enumerate(planes):
                rospy.loginfo(
                    "    plane %d: %5d pts  range %5.1f m  bearing %6.1f deg  "
                    "extent %4.1f m  z_span %4.2f m  col_span %4.2f m",
                    k, cnt, abs(dv), np.degrees(np.arctan2(nv[1], nv[0])),
                    ext, zs, cs)
            rospy.loginfo("    rejected: %s", dict(rejects))

        # Rebuild with the original header so downstream timing is unchanged.
        self.pub.publish(make_cloud(msg, out))

        # Debug clouds for RViz. ~removed is the wall points the dial discarded;
        # ~walls is everything classified as wall regardless of the dial.
        # Displaying ~output white and ~removed red shows what the dial did.
        if self.publish_debug:
            dropped = rec[wall & ~keep]
            if len(dropped):
                self.pub_removed.publish(make_cloud(msg, dropped))
            wallpts = rec[wall]
            if len(wallpts):
                self.pub_walls.publish(make_cloud(msg, wallpts))

    def report(self):
        if not self.removed_frac:
            rospy.logwarn("filter_walls: no scans processed -- check topic remap")
            return
        f = np.array(self.removed_frac)
        w = np.array(self.wall_frac)
        k = np.array(self.kept_wall_frac)
        rospy.loginfo(
            f"filter_walls summary: {self.n_scans} scans | dial "
            f"keep_fraction={self.keep_fraction:.3f} "
            f"keep_sector_deg={self.keep_sector:.1f} seed={self.seed} | "
            f"wall points {100 * w.mean():.1f}% of cloud | "
            f"wall points retained {100 * np.nanmean(k):.1f}% | "
            f"total removed {100 * f.mean():.1f}% +/- {100 * f.std():.1f}%")
        rospy.loginfo(
            "filter_walls shape tests: %.1f planes accepted per scan | "
            "candidates rejected: extent %d, z_span %d, col_span %d",
            float(np.mean(self.n_planes)) if self.n_planes else 0.0,
            self.rejects["extent"], self.rejects["z_span"],
            self.rejects["col_span"])


if __name__ == "__main__":
    rospy.init_node("filter_walls")
    WallFilter()
    rospy.loginfo("filter_walls running; remap ~input and ~output")
    rospy.spin()
