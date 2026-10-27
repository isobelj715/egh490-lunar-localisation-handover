#!/usr/bin/env python3
"""
ceiling_choice.py -- derive the level-3 height ceiling from the data instead
of picking it.

The ceiling has to sit ABOVE every regolith return and BELOW the lowest
overhead building return. 

Reads a level-2 bag -- already cropped to the retaining-wall footprint, so
only the arena is in it, but with no ceiling -- and builds a histogram of
return height in the MAP frame, which is the frame the ceiling is applied in.

Returns are split by RANGE, which is distance from the sensor, not height:

    near    inside --near metres: regolith, rocks, the retaining wall
    far     beyond --far metres: the building, the roof structure, the crane

The ceiling wants to be above the top of the NEAR population and below the
bottom of the FAR one. The script prints both histograms side by side, finds
the widest empty band between them.

Usage:
    python3 ceiling_choice.py zigzag_level2.bag \
        --poses results/testbed/zigzag_lim/est_raw_L0.txt
"""

import argparse
import sys

import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("rosbag not importable. Run: source ~/catkin_ws/devel/setup.bash")

DTYPES = {1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
          5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64}


def xyz(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(DTYPES[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats,
                   'offsets': offsets, 'itemsize': msg.point_step})
    a = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    p = np.column_stack([a['x'], a['y'], a['z']]).astype(np.float32)
    return p[np.isfinite(p).all(axis=1)]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("bag", help="a level-2 bag: arena only, no ceiling")
    ap.add_argument("--poses", required=True,
                    help="that run's unfiltered est_raw_L0.txt")
    ap.add_argument("--vertical", type=int, default=2,
                    help="column of the pose file holding height; 2 is y, "
                         "which is up in LeGO-LOAM's frame")
    ap.add_argument("--near", type=float, default=10.0,
                    help="range [m] within which returns are arena")
    ap.add_argument("--far", type=float, default=13.0,
                    help="range [m] beyond which returns are building")
    ap.add_argument("--stride", type=int, default=7,
                    help="keep 1 point in N, for memory")
    ap.add_argument("--bin", type=float, default=0.1,
                    help="histogram bin width [m]")
    ap.add_argument("--sensor-h", type=float, default=0.652,
                    help="sensor height above the regolith [m], for the "
                         "'above the sand' column")
    a = ap.parse_args()

    tr = np.loadtxt(a.poses, ndmin=2)
    t_pose, z_pose = tr[:, 0], tr[:, a.vertical]

    near, far = [], []
    n_clouds = 0
    with rosbag.Bag(a.bag) as b:
        for _topic, m, _t in b.read_messages():
            if m._type != 'sensor_msgs/PointCloud2':
                continue
            p = xyz(m)[::a.stride]
            if not len(p):
                continue
            n_clouds += 1
            stamp = m.header.stamp.to_sec()
            sz = float(np.interp(stamp, t_pose, z_pose))
            r = np.linalg.norm(p, axis=1)
            mz = sz + p[:, 2]            # map height, roll/pitch ignored
            near.append(mz[r < a.near])
            far.append(mz[r > a.far])

    if not n_clouds:
        sys.exit("no PointCloud2 messages in %s" % a.bag)
    near = np.concatenate(near) if near else np.array([])
    far = np.concatenate(far) if far else np.array([])
    if not len(near) or not len(far):
        sys.exit("one of the two populations is empty -- adjust --near/--far")

    lo = float(min(near.min(), far.min()))
    hi = float(max(near.max(), far.max()))
    edges = np.arange(lo, hi + a.bin, a.bin)
    hn, _ = np.histogram(near, bins=edges)
    hf, _ = np.histogram(far, bins=edges)

    print("\n%s  --  %d clouds, 1 point in %d kept"
          % (a.bag, n_clouds, a.stride))
    print("   near  = within %.0f m of the sensor: %d points" % (a.near, len(near)))
    print("   far   = beyond %.0f m: %d points" % (a.far, len(far)))
    print("   heights are in the MAP frame; 0.0 is the start pose, so the "
          "regolith\n   under the rover at the start is about %+.2f m"
          % -a.sensor_h)

    print("\n%8s %10s %12s %12s   %s"
          % ("map z", "above sand", "near", "far", ""))
    print("-" * 72)
    scale = max(hn.max(), hf.max())
    for i in range(len(hn)):
        if hn[i] == 0 and hf[i] == 0:
            continue
        bar_n = "#" * int(18 * hn[i] / scale)
        bar_f = "*" * int(18 * hf[i] / scale)
        print("%8.2f %10.2f %12d %12d   %-18s|%s"
              % (edges[i], edges[i] + a.sensor_h, hn[i], hf[i], bar_n, bar_f))

    # The top of the arena population and the bottom of the building one.
    near_top = float(np.percentile(near, 99.99))
    far_bottom = float(np.percentile(far, 0.01))
    print("\n   top of the NEAR population (99.99th pct): %+.2f m  "
          "(%.2f m above the sand)" % (near_top, near_top + a.sensor_h))
    print("   bottom of the FAR population (0.01th pct): %+.2f m  "
          "(%.2f m above the sand)" % (far_bottom, far_bottom + a.sensor_h))

    if far_bottom > near_top:
        mid = 0.5 * (near_top + far_bottom)
        print("\n   The two populations are SEPARATED by %.2f m of empty "
              "height." % (far_bottom - near_top))
        print("   A ceiling anywhere in [%.2f, %.2f] removes every building "
              "return\n   and no arena return. Midpoint: ZHI = %+.2f"
              % (near_top, far_bottom, mid))
        print("   That is the value to quote, with this histogram as its "
              "justification.")
    else:
        print("\n   !! The populations OVERLAP by %.2f m. No flat ceiling can "
              "separate\n      them: any value that removes all the building "
              "also removes arena\n      returns, and vice versa. Report that, "
              "and either range-gate the\n      crop or state the overlap as a "
              "limitation." % (near_top - far_bottom))

    print("""
   Roll and pitch are ignored in the map-height reconstruction, so these
   heights carry a few centimetres of error at long range.""")


if __name__ == "__main__":
    main()
