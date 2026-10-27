#!/usr/bin/env python3
"""
Check the cropped filtered bags against what the filter actually requires.

What the filter needs is:

  1. NO INTERIOR GAPS. A run of missing scans mid-recording leaves the odometry
     with nothing to track across, and the resulting divergence is
     indistinguishable from the structure loss being measured. 

  2. IDENTICAL STAMPS ACROSS LEVELS. The levels must differ only by their crop
     boxes. If each crop pass lost a different set of scans, the levels differ by
     their inputs too, and the comparison is not controlled. 

Usage:
    python3 verify_levels.py 2026-09-16-07-03-07.bag level1.bag level2.bag level3.bag
"""

import os
import struct
import sys

import numpy as np

try:
    import rosbag
except ImportError:
    sys.exit("rosbag not importable. Run: source ~/catkin_ws/devel/setup.bash")


def stamps_of(path):
    """Header stamps in milliseconds, as ints, from the first PointCloud2 topic."""
    bag = rosbag.Bag(path)
    info = bag.get_type_and_topic_info()[1]
    topic = next((t for t, v in info.items()
                  if v.msg_type == "sensor_msgs/PointCloud2"), None)
    out = []
    if topic:
        for _, (dt, data, md5, pos, pt), _t in bag.read_messages(topics=[topic],
                                                                raw=True):
            _, sec, nsec = struct.unpack_from("<III", data, 0)
            out.append(int(round((sec + nsec * 1e-9) * 1000)))
    bag.close()
    return topic, np.array(sorted(out), dtype=np.int64)


def main():
    if len(sys.argv) < 3:
        sys.exit(__doc__)
    src_path, level_paths = sys.argv[1], sys.argv[2:]

    _, src = stamps_of(src_path)
    if len(src) == 0:
        sys.exit("no clouds in %s" % src_path)
    print("")
    print("source %s: %d clouds, %.2f s"
          % (os.path.basename(src_path), len(src), (src[-1] - src[0]) / 1000.0))

    levels, ok = {}, True
    print("")
    print("%-16s %7s %8s %10s %-22s" %
          ("bag", "clouds", "missing", "max gap", "missing scans are"))
    print("-" * 70)

    for p in level_paths:
        if not os.path.exists(p):
            print("%-16s %7s %8s %10s %s" % (p, "-", "-", "-", "FILE MISSING"))
            ok = False
            continue
        _, st = stamps_of(p)
        levels[p] = st
        if len(st) < 2:
            print("%-16s %7d %8s %10s %s" % (p, len(st), "-", "-", "TOO FEW"))
            ok = False
            continue

        d = np.diff(st)
        med = int(np.median(d))
        mx = int(d.max())
        interior = mx > 1.5 * med

        # where in the source's ordering are the absent scans?
        present = np.isin(src, st)
        absent = np.flatnonzero(~present)
        n_missing = len(absent)
        if n_missing == 0:
            where = "none"
        else:
            head = int((absent < 5).sum())
            tail = int((absent >= len(src) - 5).sum())
            mid = n_missing - head - tail
            bits = []
            if head:
                bits.append("%d at start" % head)
            if tail:
                bits.append("%d at end" % tail)
            if mid:
                bits.append("%d INTERIOR" % mid)
            where = ", ".join(bits)

        bad = interior or (mid if n_missing else 0)
        ok = ok and not bad
        print("%-16s %7d %8d %9.3fs %-22s %s"
              % (p, len(st), n_missing, mx / 1000.0, where,
                 "FAIL" if bad else "ok"))

    # ---- do the levels hold the same scans? --------------------------
    print("")
    names = list(levels)
    if len(names) > 1:
        ref = levels[names[0]]
        same = all(len(levels[n]) == len(ref) and np.array_equal(levels[n], ref)
                   for n in names[1:])
        if same:
            print("All levels hold the IDENTICAL %d scans -- they differ only by"
                  % len(ref))
            print("their crop boxes, which is what the ablation assumes.")
        else:
            ok = False
            print("LEVELS DIFFER IN WHICH SCANS THEY HOLD:")
            common = ref
            for n in names[1:]:
                common = np.intersect1d(common, levels[n])
            for n in names:
                extra = len(levels[n]) - len(common)
                print("   %-16s %d scans, %d not shared" % (n, len(levels[n]),
                                                            extra))
            print("   %d scans common to all." % len(common))
            print("   The levels differ by their inputs as well as their crop")
            print("   boxes, so the comparison is not controlled. Re-crop.")

    print("")
    if ok:
        print("VERDICT: usable. Run step5_final.sh.")
        print("A few scans absent from the start of every bag is expected -- the")
        print("crop node cannot transform a scan before tf has published, and it")
        print("affects every level identically.")
    else:
        print("VERDICT: not usable.")
        print("An INTERIOR gap, or levels holding different scans, will produce")
        print("divergence that looks like a structure effect. Re-crop with")
        print("TF_TIMEOUT=5.0 FORCE=1 ./step0_recrop.sh")


if __name__ == "__main__":
    main()
