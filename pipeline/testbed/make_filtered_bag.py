#!/usr/bin/env python3
"""
Apply the wall filter to a bag OFFLINE and write a new bag.

Usage:
    # passthrough control: walls identified, all kept. Must reproduce L1.
    python3 make_filtered_bag.py level1.bag fix100.bag --keep-fraction 1.0

    # walls removed
    python3 make_filtered_bag.py level1.bag fix000.bag --keep-fraction 0.0

Then replay either exactly like a level bag:
    rosbag play --clock --rate 0.3 fix000.bag \
        /velodyne_points_cropped:=/velodyne_points
"""

import argparse
import os
import sys
import time

import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("rosbag not importable. Run: source ~/catkin_ws/devel/setup.bash")


def load_filter(path):
    d = os.path.dirname(os.path.abspath(path))
    sys.path.insert(0, d)
    try:
        import filter_walls
    except Exception as e:
        sys.exit("could not import filter_walls from %s: %s" % (d, e))
    if not hasattr(filter_walls, "column_span"):
        sys.exit("that filter_walls.py is the OLD copy (no column_span); it "
                 "mislabels ground as wall.")
    return filter_walls


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("inbag")
    ap.add_argument("outbag")
    ap.add_argument("--topic", default=None,
                    help="default: the first PointCloud2 topic in the bag")
    ap.add_argument("--out-topic", default=None,
                    help="default: same as the input topic, so the output "
                         "replays with the same remap as a level bag")
    ap.add_argument("--filter", default=os.path.expanduser(
        "~/catkin_ws/pipeline/testbed/filter_walls.py"))
    ap.add_argument("--keep-fraction", type=float, required=True,
                    help="1.0 = passthrough control, 0.0 = all walls removed")
    ap.add_argument("--keep-sector-deg", type=float, default=360.0)
    ap.add_argument("--sector-center-deg", type=float, default=0.0)
    ap.add_argument("--seed", type=int, default=1)
    ap.add_argument("--dist-thresh", type=float, default=0.08)
    ap.add_argument("--min-inliers", type=int, default=400)
    ap.add_argument("--max-planes", type=int, default=12)
    ap.add_argument("--iters", type=int, default=400)
    ap.add_argument("--min-range", type=float, default=2.0)
    ap.add_argument("--min-extent", type=float, default=1.0)
    ap.add_argument("--min-z-span", type=float, default=0.20)
    ap.add_argument("--min-col-span", type=float, default=0.10)
    args = ap.parse_args()

    if os.path.exists(args.outbag):
        sys.exit("%s exists -- delete it or choose another name" % args.outbag)

    fw = load_filter(args.filter)
    kf = min(max(args.keep_fraction, 0.0), 1.0)

    # Separate streams so changing keep_fraction does not perturb which planes
    # are found -- same arrangement the node uses.
    ss = np.random.SeedSequence(args.seed).spawn(2)
    rng, rng_keep = np.random.default_rng(ss[0]), np.random.default_rng(ss[1])

    bag = rosbag.Bag(args.inbag)
    info = bag.get_type_and_topic_info()[1]
    topic = args.topic or next(
        (t for t, v in info.items() if v.msg_type == "sensor_msgs/PointCloud2"),
        None)
    if topic is None:
        sys.exit("no PointCloud2 topic in %s" % args.inbag)
    out_topic = args.out_topic or topic
    total = info[topic].message_count

    print("")
    print("in   %s  (%s, %d clouds)" % (args.inbag, topic, total))
    print("out  %s  (%s)" % (args.outbag, out_topic))
    print("keep_fraction %.3f  seed %d  min_inliers %d  max_planes %d"
          % (kf, args.seed, args.min_inliers, args.max_planes))
    if kf >= 1.0:
        print("PASSTHROUGH CONTROL: walls are identified and all kept, so this")
        print("bag must give the same result as the unfiltered one.")
    print("")

    out = rosbag.Bag(args.outbag, "w")
    n, removed, wallfrac, nplanes = 0, [], [], []
    t0 = time.time()
    try:
        for _tp, msg, t in bag.read_messages(topics=[topic]):
            rec = fw.as_record_array(msg)
            xyz = np.column_stack([rec["x"].astype(np.float64),
                                   rec["y"].astype(np.float64),
                                   rec["z"].astype(np.float64)])
            fin = np.isfinite(xyz).all(axis=1)
            rec, xyz = rec[fin], xyz[fin]

            if len(rec) < args.min_inliers:
                out.write(out_topic, fw.make_cloud(msg, rec), t)
                n += 1
                continue

            r = np.linalg.norm(xyz[:, :2], axis=1)
            valid = r > args.min_range

            wall = np.zeros(len(xyz), dtype=bool)
            sub, planes, _rej = fw.fit_vertical_planes(
                xyz[valid], args.dist_thresh, args.min_inliers,
                args.max_planes, args.iters, 8.0, rng, 6000,
                args.min_extent, args.min_z_span, args.min_col_span)
            wall[np.flatnonzero(valid)] = sub

            if wall.any():
                survive = fw.sector_mask(xyz, args.sector_center_deg,
                                         args.keep_sector_deg)
                if kf <= 0.0:
                    survive = np.zeros(len(xyz), dtype=bool)
                elif kf < 1.0:
                    survive &= rng_keep.random(len(xyz)) < kf
                keep = ~wall | survive
            else:
                keep = np.ones(len(xyz), dtype=bool)

            kept = rec[keep]
            if len(kept) == 0:          # never emit an empty cloud
                kept = rec
            out.write(out_topic, fw.make_cloud(msg, kept), t)

            n += 1
            removed.append(1.0 - len(kept) / len(rec))
            wallfrac.append(wall.mean())
            nplanes.append(len(planes))
            if n % 50 == 0:
                el = time.time() - t0
                print("  %4d/%d  %.0f%% removed  %.1f planes  "
                      "(%.1f scans/s, ~%.0f s left)"
                      % (n, total, 100 * np.mean(removed[-50:]),
                         np.mean(nplanes[-50:]), n / el,
                         (total - n) / max(n / el, 1e-6)))
    finally:
        out.close()
        bag.close()

    print("")
    print("wrote %s: %d clouds in %.0f s" % (args.outbag, n, time.time() - t0))
    if removed:
        rm, wf = np.array(removed), np.array(wallfrac)
        print("  wall points %.1f%% of cloud | removed %.1f%% +/- %.1f%% | "
              "%.1f planes/scan"
              % (100 * wf.mean(), 100 * rm.mean(), 100 * rm.std(),
                 float(np.mean(nplanes))))
    if n != total:
        print("  WARNING: %d clouds in, %d out -- they should match" % (total, n))
    else:
        print("  cloud count matches the input, so this bag replays exactly "
              "like a level bag.")


if __name__ == "__main__":
    main()
