#!/usr/bin/env python3
"""
Error against the degeneracy threshold, with the scene's own lambda_min marked.

Usage:
    python3 threshold_sweep.py --run rectangle --level L2 \
        --structure results/testbed/rectangle/logs/L2/structure.csv \
        --plot threshold_sweep.png

    python3 threshold_sweep.py --run rectangle --lambda-min 28.4 \
        --point 75 results/testbed/odd_build/est_raw_L2.txt \
        --plot threshold_sweep.png
"""

import argparse
import csv
import glob
import os
import re
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

INK = "#16212B"
MUTED = "#566370"
GRID = "#D8D5CE"
PAPER = "#FAF9F6"
STOCK = "#C4521F"
FIXED = "#1B6CA8"

STOCK_MAP = 100.0        # eignThre in mapOptmization.cpp as shipped
STOCK_FEATURE = 10.0     # eignThre in featureAssociation.cpp as shipped


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        return None
    return a[:, 0], a[:, 1:4]


def plane_of(p):
    v = int(np.argmin(np.ptp(p, axis=0)))
    return [i for i in range(3) if i != v]


def final_error(base, est):
    """Separation at the last shared timestamp -- brake_effect.py's measure."""
    b, e = load(base), load(est)
    if b is None or e is None:
        return None
    tb, pb = b
    te, pe = e
    t_end = min(tb[-1], te[-1])
    if t_end <= max(tb[0], te[0]):
        return None
    plane = plane_of(pb)
    at = lambda t, p: np.array([np.interp(t_end, t, p[:, k]) for k in plane])
    return float(np.linalg.norm(at(tb, pb) - at(te, pe)))


def median_lambda(path, column="map_full_lambda_min"):
    vals = []
    with open(path, newline="") as fh:
        for row in csv.DictReader(fh):
            try:
                vals.append(float(row[column]))
            except (KeyError, TypeError, ValueError):
                continue
    return float(np.median(vals)) if vals else None


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="rectangle",
                    help="stock run directory name under --root")
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--level", default="L2")
    ap.add_argument("--feature", type=float, default=STOCK_FEATURE,
                    help="hold the featureAssociation threshold at this "
                         "value and sweep the map threshold")
    ap.add_argument("--point", nargs=2, action="append", default=[],
                    metavar=("THRESHOLD", "FILE"),
                    help="a sweep point given explicitly; repeatable")
    ap.add_argument("--lambda-min", type=float,
                    help="the scene's median lambda_min, for the marker line")
    ap.add_argument("--structure",
                    help="structure.csv to take lambda_min from instead")
    ap.add_argument("--no-stock", action="store_true",
                    help="do not use the plain <run> directory as the 100 "
                         "point")
    ap.add_argument("--floor", type=float, default=0.012,
                    help="noise floor, shaded and unlabelled; 0 to omit")
    ap.add_argument("--plot", default="threshold_sweep.png")
    args = ap.parse_args()

    stock_dir = os.path.join(args.root, args.run)
    base = os.path.join(stock_dir, "est_raw_L0.txt")
    if not os.path.exists(base):
        sys.exit("no unfiltered baseline at %s -- every point is measured "
                 "against it" % base)

    points, notes = [], []
    for thr, path in args.point:
        if os.path.exists(path):
            points.append((float(thr), path, "given"))
        else:
            notes.append("%s: not found" % path)

    pat = re.compile(r"^%s_m(\d+)f(\d+)$" % re.escape(args.run))
    for d in sorted(glob.glob(os.path.join(args.root, "%s_*" % args.run))):
        m = pat.match(os.path.basename(os.path.normpath(d)))
        if not m:
            continue
        mapthr, feat = float(m.group(1)), float(m.group(2))
        traj = os.path.join(d, "est_raw_%s.txt" % args.level)
        if not os.path.exists(traj):
            notes.append("%s: no %s pass" % (os.path.basename(d), args.level))
            continue
        if feat != args.feature:
            notes.append("%s: feature stage at %g, not %g -- left off the "
                         "plot so only one thing varies along the axis"
                         % (os.path.basename(d), feat, args.feature))
            continue
        points.append((mapthr, traj, "m%gf%g" % (mapthr, feat)))

    have = {p[0] for p in points}
    if (not args.no_stock and STOCK_MAP not in have
            and args.feature == STOCK_FEATURE):
        traj = os.path.join(stock_dir, "est_raw_%s.txt" % args.level)
        if os.path.exists(traj):
            points.append((STOCK_MAP, traj, "stock build"))
        else:
            notes.append("%s: no stock %s pass" % (args.run, args.level))

    if not points:
        sys.exit("no sweep points for %s at %s.\n%s\n\nExpected directories "
                 "like %s/%s_m0f%g. If the sweep was written somewhere else, "
                 "pass each point with --point."
                 % (args.run, args.level, "\n".join("  ! " + n for n in notes),
                    args.root, args.run, args.feature))

    rows = []
    for thr, path, how in sorted(points):
        err = final_error(base, path)
        if err is None:
            notes.append("%s: could not measure" % os.path.relpath(path))
            continue
        rows.append((thr, err, path, how))
    if not rows:
        sys.exit("nothing measurable.\n" + "\n".join(notes))

    lam = args.lambda_min
    if lam is None and args.structure:
        lam = median_lambda(args.structure)

    hdr = "%10s %12s  %-12s %s" % ("eignThre", "error", "source", "file")
    print("\n%s, level %s: error at end of run against the map threshold\n"
          % (args.run, args.level))
    print(hdr)
    print("-" * len(hdr))
    for thr, err, path, how in rows:
        print("%10g %10.3f m  %-12s %s" % (thr, err, how,
                                           os.path.relpath(path)))
    for n in notes:
        print("  ! " + n)
    if lam is not None:
        below = [r for r in rows if r[0] < lam]
        above = [r for r in rows if r[0] >= lam]
        print("\nscene median lambda_min: %.1f" % lam)
        if below:
            print("below it  (%d points): %.3f to %.3f m"
                  % (len(below), min(r[1] for r in below),
                     max(r[1] for r in below)))
        if above:
            print("at/above  (%d points): %.3f to %.3f m"
                  % (len(above), min(r[1] for r in above),
                     max(r[1] for r in above)))

    x = [r[0] for r in rows]
    y = [r[1] for r in rows]

    fig, ax = plt.subplots(figsize=(9.5, 5.4), facecolor=PAPER)
    ax.set_facecolor(PAPER)
    if args.floor > 0:
        ax.axhspan(1e-9, args.floor, color=MUTED, alpha=0.18, zorder=0)

    if lam is not None:
        ax.axvline(lam, color=MUTED, lw=1.4, ls=(0, (4, 3)), zorder=1)
        ax.text(lam, max(y) * 2.4, "  scene $\\lambda_{min}$ = %.1f" % lam,
                color=MUTED, fontsize=12, ha="left", va="top")

    ax.plot(x, y, "-", color=MUTED, lw=1.4, zorder=2)
    span = (max(x) - min(x)) or 1.0
    prev_x, high = None, False
    for xi, yi in zip(x, y):
        braked = lam is not None and xi >= lam
        ax.plot([xi], [yi], "o", ms=10, zorder=3,
                color=STOCK if braked else FIXED)
        # Sweeps bunch at the low end, where neighbouring labels collide.
        # Lift every second one in a crowded stretch rather than shrinking it.
        high = (prev_x is not None and (xi - prev_x) < 0.12 * span
                and not high)
        ax.annotate("%.3f m" % yi if yi < 1 else "%.2f m" % yi,
                    (xi, yi), textcoords="offset points",
                    xytext=(0, 30 if high else 14),
                    ha="center", fontsize=12, color=INK)
        prev_x = xi
    ax.set_xlim(min(x) - 0.06 * span, max(x) + 0.06 * span)

    ax.set_yscale("log")
    lo = min(y) * 0.35
    if args.floor > 0:
        lo = min(lo, args.floor * 0.6)
    ax.set_ylim(lo, max(y) * 4.5)
    ax.set_xlabel("eignThre in mapOptmization.cpp  (shipped value: 100)")
    ax.set_ylabel("error at end of run [m]")
    ax.grid(True, which="both", lw=0.7, color=GRID)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK)
    ax.xaxis.label.set_color(INK)
    ax.yaxis.label.set_color(INK)
    fig.tight_layout()
    fig.savefig(args.plot, dpi=170)
    print("\nwrote %s" % args.plot)


if __name__ == "__main__":
    main()
