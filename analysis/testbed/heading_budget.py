#!/usr/bin/env python3
"""
heading_budget.py -- split the position error into the part heading explains
and the part it does not.

Take the estimate's own motion one step at a time. Each step is a small
displacement expressed in the estimate's (wrong) frame. Rotate that step back
by the heading error measured at that moment, and re-integrate. What comes out
is the trajectory the pipeline would have produced with a perfect heading
reference and the same scan matching. Whatever error survives that correction
is NOT heading -- it is translation error, scale error, everything else.

  total error            what the pipeline actually produced
  after correction       what remains once heading is perfect
  explained by heading   the difference
  
Usage:
    python3 heading_budget.py \
        --ref results/testbed/zigzag_lim/est_raw_L0.txt \
        --est results/testbed/zigzag_lim_m0f10/est_raw_L3.txt
"""

import argparse
import sys

import numpy as np


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 8:
        sys.exit("%s has %d columns; need TUM (t x y z qx qy qz qw)"
                 % (path, a.shape[1]))
    q = a[:, 4:8]
    q = q / np.linalg.norm(q, axis=1, keepdims=True)
    x, y, z, w = q[:, 0], q[:, 1], q[:, 2], q[:, 3]
    R = np.empty((len(q), 3, 3))
    R[:, 0, 0] = 1 - 2 * (y * y + z * z)
    R[:, 0, 1] = 2 * (x * y - z * w)
    R[:, 0, 2] = 2 * (x * z + y * w)
    R[:, 1, 0] = 2 * (x * y + z * w)
    R[:, 1, 1] = 1 - 2 * (x * x + z * z)
    R[:, 1, 2] = 2 * (y * z - x * w)
    R[:, 2, 0] = 2 * (x * z - y * w)
    R[:, 2, 1] = 2 * (y * z + x * w)
    R[:, 2, 2] = 1 - 2 * (x * x + y * y)
    return a[:, 0], a[:, 1:4], R


def heading(R, axis, plane):
    """Unwrapped ground-plane angle of a body axis. Full series only."""
    v = R[:, :, axis]
    return np.unwrap(np.arctan2(v[:, plane[1]], v[:, plane[0]]))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--ref", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--axis", default="z", choices=["x", "y", "z"],
                    help="body axis taken as forward; 'z' for LeGO-LOAM, and "
                         "diagnose_heading.py confirms it per run")
    ap.add_argument("--vertical", choices=["x", "y", "z"], default=None)
    a = ap.parse_args()

    tr, pr, Rr = load(a.ref)
    te, pe, Re = load(a.est)

    lo, hi = max(tr[0], te[0]), min(tr[-1], te[-1])
    sel = (te >= lo) & (te <= hi)
    te, pe, Re = te[sel], pe[sel], Re[sel]
    if len(te) < 10:
        sys.exit("the files barely overlap in time")

    axn = "xyz"
    spread = np.ptp(pr, axis=0)
    vert = axn.index(a.vertical) if a.vertical else int(np.argmin(spread))
    plane = [i for i in range(3) if i != vert]
    fwd = axn.index(a.axis)

    hr = heading(Rr, fwd, plane)
    he = heading(Re, fwd, plane)
    th = he - np.interp(te, tr, hr)

    # Reference position at those same timestamps.
    pri = np.column_stack([np.interp(te, tr, pr[:, k]) for k in range(3)])

    # Re-integrate the estimate's steps with each one rotated back by the
    # heading error measured at that step.
    corr = np.array(pe, dtype=float, copy=True)
    for i in range(1, len(te)):
        d = pe[i] - pe[i - 1]
        c, s = np.cos(-th[i]), np.sin(-th[i])
        u, v = d[plane[0]], d[plane[1]]
        step = np.zeros(3)
        step[plane[0]] = c * u - s * v
        step[plane[1]] = s * u + c * v
        step[vert] = d[vert]
        corr[i] = corr[i - 1] + step

    err_raw = np.linalg.norm(pe - pri, axis=1)
    err_cor = np.linalg.norm(corr - pri, axis=1)
    dist = np.linalg.norm(pri - pri[0], axis=1)

    print("\nref  %s\nest  %s" % (a.ref, a.est))
    print("  %d poses over %.1f s, forward axis '%s', vertical '%s'"
          % (len(te), hi - lo, a.axis, axn[vert]))
    print("  path of the reference %.2f m, furthest from start %.2f m"
          % (np.abs(np.diff(dist)).sum() + 0.0, dist.max()))
    print("  heading error at the end %.1f°, worst %.1f°"
          % (np.degrees(th[-1]), np.degrees(th[np.abs(th).argmax()])))

    print("\nposition error, metres\n")
    print("%-28s %10s %10s" % ("", "at the end", "worst"))
    print("-" * 52)
    print("%-28s %10.2f %10.2f" % ("as recorded", err_raw[-1], err_raw.max()))
    print("%-28s %10.2f %10.2f" % ("with heading corrected",
                                   err_cor[-1], err_cor.max()))
    share = 1.0 - err_cor[-1] / max(err_raw[-1], 1e-9)
    print("%-28s %9.0f%%" % ("explained by heading", 100 * share))

    print("\nwhere it stands through the run\n")
    print("   %6s %8s %12s %12s %10s"
          % ("", "t", "dist", "as recorded", "corrected"))
    for f in (0.25, 0.50, 0.75, 0.90, 1.0):
        i = min(int(f * (len(te) - 1)), len(te) - 1)
        print("   %4.0f%%  %6.1f s %10.2f m %10.2f m %9.2f m"
              % (100 * f, te[i] - te[0], dist[i], err_raw[i], err_cor[i]))

    if share > 0.8:
        verdict = ("Heading accounts for nearly all of it. A heading "
                   "reference would recover the run.")
    elif share > 0.4:
        verdict = ("Heading is the largest single contributor, but a real "
                   "fraction is translation error\n  that a heading "
                   "reference would not touch.")
    elif share > 0.05:
        verdict = ("Heading is a minor contributor. Most of this error is "
                   "not rotational.")
    else:
        verdict = ("Heading explains essentially none of it. The error is "
                   "translational.")
    print("\n  %s" % verdict)



if __name__ == "__main__":
    main()
