#!/usr/bin/env python3
"""
noise_floor.py -- reproduce the replay noise floor from stored repeat runs.

WHAT THE NUMBER IS
Identical input, identical parameters, identical build -- run N times. Nothing
in the pipeline is random, so any difference between the runs is the pipeline
being non-deterministic under VM timing: mapOptmization processes whatever
scans have arrived, and its optimisation sequence depends on CPU scheduling.

Usage:
    python3 noise_floor.py results/testbed/rep03
    python3 noise_floor.py results/testbed/rep03 --exclude R1
"""

import argparse
import glob
import os
import sys

import numpy as np


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        sys.exit("%s: need at least 4 columns (t x y z ...)" % path)
    return a[:, 0], a[:, 1:4]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("dir", nargs="?", default="results/testbed/rep03")
    ap.add_argument("--pattern", default="est_raw_R*.txt")
    ap.add_argument("--exclude", action="append", default=[],
                    help="repeat name to leave out, e.g. R1; repeatable")
    ap.add_argument("--rate", type=float, default=5.0,
                    help="grid rate [Hz] the runs are interpolated onto")
    ap.add_argument("--vertical", choices=["x", "y", "z"], default=None,
                    help="force the vertical axis instead of measuring it")
    a = ap.parse_args()

    files = sorted(glob.glob(os.path.join(a.dir, a.pattern)))
    runs = []
    for f in files:
        name = os.path.basename(f)[len("est_raw_"):-len(".txt")]
        if name in a.exclude:
            continue
        t, p = load(f)
        runs.append((name, t, p))

    if len(runs) < 2:
        sys.exit("found %d usable repeat(s) in %s -- need at least 2"
                 % (len(runs), a.dir))

    print("\n%d repeats from %s" % (len(runs), a.dir))
    if a.exclude:
        print("   excluded: %s" % ", ".join(a.exclude))

    # Coverage first. Identical runs that cover different amounts of the
    # recording are already evidence of the non-determinism being measured.
    print("\n%-6s %8s %10s %8s %14s" % ("repeat", "poses", "duration", "Hz",
                                        "path len [m]"))
    durs = []
    for n, t, p in runs:
        d = float(t[-1] - t[0])
        durs.append(d)
        plen = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
        print("%-6s %8d %9.1fs %8.2f %14.2f"
              % (n, len(t), d, len(t) / max(d, 1e-6), plen))
    if max(durs) - min(durs) > 2.0:
        print("   !! coverage differs by %.1f s across identical runs. The "
              "shared window below\n      is set by the shortest of them."
              % (max(durs) - min(durs)))

    # Ground plane. step4_repeats.sh hard-coded columns 0 and 2, which is
    # right for LeGO-LOAM's frame (y up) -- check rather than assume.
    allp = np.vstack([p for _, _, p in runs])
    spread = np.ptp(allp, axis=0)
    vert = "xyz".index(a.vertical) if a.vertical else int(np.argmin(spread))
    plane = [i for i in range(3) if i != vert]
    print("\n   spread  x %.2f  y %.2f  z %.2f m -> vertical '%s'%s, "
          "ground plane %s-%s"
          % (spread[0], spread[1], spread[2], "xyz"[vert],
             " (forced)" if a.vertical else "", "xyz"[plane[0]],
             "xyz"[plane[1]]))
    if vert != 1:
        print("   !! the original computation assumed y was vertical. It is "
              "not here, so this\n      result is NOT comparable to the "
              "stored floor without explanation.")

    t0 = max(t[0] for _, t, _ in runs)
    t1 = min(t[-1] for _, t, _ in runs)
    n_grid = max(int((t1 - t0) * a.rate), 8)
    grid = np.linspace(t0, t1, n_grid)
    P = {n: np.column_stack([np.interp(grid, t, p[:, k]) for k in (0, 1, 2)])
         for n, t, p in runs}

    print("\npairwise separation between IDENTICAL runs, over %.1f s "
          "(%d grid points)\n" % (t1 - t0, n_grid))
    print("%-14s %9s %9s %9s %9s" % ("pair", "median", "p95", "max", "final"))
    print("-" * 54)
    meds = []
    names = [n for n, _, _ in runs]
    for i in range(len(names)):
        for j in range(i + 1, len(names)):
            d = P[names[i]] - P[names[j]]
            dist = np.linalg.norm(d[:, plane], axis=1)
            meds.append(float(np.median(dist)))
            print("%-14s %9.3f %9.3f %9.3f %9.3f"
                  % ("%s vs %s" % (names[i], names[j]), np.median(dist),
                     np.percentile(dist, 95), dist.max(), dist[-1]))

    floor = float(np.mean(meds))
    print("\n   NOISE FLOOR = mean pairwise median separation = %.3f m"
          % floor)
    print("   over %d pair(s) of %d repeats, %.1f s shared window"
          % (len(meds), len(runs), t1 - t0))
    print("""
   Nothing smaller than this can be attributed to a manipulation, at any
   number of filtered levels. """)


if __name__ == "__main__":
    main()
