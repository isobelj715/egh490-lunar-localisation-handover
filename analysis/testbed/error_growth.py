#!/usr/bin/env python3
"""
How the error accumulates across a run, against distance travelled.

WHAT IS MEASURED
Separation from that run's unfiltered L0 pass, in the ground plane, at every
timestamp they share -- the same quantity brake_effect.py reports at the last
one. L0 carries its own error, so every value is a LOWER BOUND; the bound
applies equally to both conditions, which is what the comparison needs.

Usage:
    python3 error_growth.py --run rectangle --level L2 --plot growth_L2.png
    python3 error_growth.py --run rectangle --levels L2 L3 --plot growth.png
    python3 error_growth.py --run zigzag --level L3 --only removed \
        --plot zigzag_L3_growth.png
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402

INK = "#16212B"
MUTED = "#566370"
GRID = "#D8D5CE"
PAPER = "#FAF9F6"
STILL = "#D8D5CE"

LEVEL_HUE = {"L0": "#566370", "L1": "#7A8691",
             "L2": "#1B6CA8", "L3": "#8A3412"}
LEVEL_PALE = {"L0": "#9AA5AE", "L1": "#B3BBC2",
              "L2": "#8FB8D8", "L3": "#C99B86"}
CONDITION_HUE = {
    ("L0", "control"): "#566370",
    ("L2", "stock"):   "#C4521F",
    ("L3", "stock"):   "#8A3412",
    ("L2", "removed"): "#1B6CA8",
    ("L3", "removed"): "#103F63",
}


def series_style(level, condition, colour_by):
    """Colour and dash pattern for one (level, condition) series."""
    if colour_by == "condition":
        return CONDITION_HUE.get((level, condition), "#566370"), "-"
    colour = LEVEL_HUE.get(level, "#566370")
    return colour, "--" if condition == "stock" else "-"

SPARE = ["#C4521F", "#1B6CA8", "#8A3412", "#8FB8D8", "#566370"]


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        return None
    return a[:, 0], a[:, 1:4]


def plane_of(p):
    v = int(np.argmin(np.ptp(p, axis=0)))
    return [i for i in range(3) if i != v]


def arc_length(xy):
    return np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(xy, axis=0), axis=1))]


def still_mask(t, xy, thr):
    step = np.r_[0.0, np.linalg.norm(np.diff(xy, axis=0), axis=1)]
    dt = np.r_[1e-6, np.diff(t)]
    return (step / np.maximum(dt, 1e-6)) <= thr


def spans(mask, x):
    """Contiguous True runs of `mask`, as (x_start, x_end) pairs."""
    out, i, n = [], 0, len(mask)
    while i < n:
        if mask[i]:
            j = i
            while j + 1 < n and mask[j + 1]:
                j += 1
            out.append((x[i], x[j]))
            i = j + 1
        else:
            i += 1
    return out


def curve(base, est, use_time):
    """Separation against distance travelled (or time), on shared samples."""
    b, e = load(base), load(est)
    if b is None or e is None:
        return None
    tb, pb = b
    te, pe = e
    lo, hi = max(tb[0], te[0]), min(tb[-1], te[-1])
    if hi <= lo:
        return None
    t = tb[(tb >= lo) & (tb <= hi)]
    if t.size == 0:
        return None
    if t[-1] < hi:
        t = np.r_[t, hi]
    plane = plane_of(pb)
    bxy = np.column_stack([np.interp(t, tb, pb[:, k]) for k in plane])
    exy = np.column_stack([np.interp(t, te, pe[:, k]) for k in plane])
    err = np.linalg.norm(bxy - exy, axis=1)
    x = (t - t[0]) if use_time else arc_length(bxy)
    return x, err, t, bxy


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--run", default="rectangle")
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--level", help="a single level; shorthand for --levels")
    ap.add_argument("--levels", nargs="+", default=["L2"])
    ap.add_argument("--only", choices=["stock", "removed", "both"],
                    default="both")
    ap.add_argument("--suffixes", nargs="+",
                    default=["m0f10", "m10f10", "m0f0", "nobrake"])
    ap.add_argument("--series", nargs=2, action="append", default=[],
                    metavar=("LABEL", "FILE"),
                    help="an extra curve, given explicitly; repeatable")
    ap.add_argument("--baseline", help="override the L0 reference")
    ap.add_argument("--x", choices=["distance", "time"], default="distance")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="m/s below which the rover counts as stationary")
    ap.add_argument("--floor", type=float, default=0.012)
    ap.add_argument("--colour-by", choices=["level", "condition"],
                    default="level",
                    help="level: blue L2 / burnt orange L3, dashed for "
                         "stock. "
                         "condition: the bar charts' orange-and-blue")
    ap.add_argument("--linear", action="store_true",
                    help="linear y-axis; the default is log, which is the "
                         "only way 0.03 m and 12 m share an axis")
    ap.add_argument("--plot", default="error_growth.png")
    args = ap.parse_args()

    levels = [args.level] if args.level else args.levels
    stock_dir = os.path.join(args.root, args.run)
    base = args.baseline or os.path.join(stock_dir, "est_raw_L0.txt")
    if not os.path.exists(base):
        sys.exit("no baseline at %s -- every curve is measured against it"
                 % base)

    series, notes = [], []
    for lvl in levels:
        if args.only in ("stock", "both"):
            p = os.path.join(stock_dir, "est_raw_%s.txt" % lvl)
            if os.path.exists(p):
                series.append(("%s stock" % lvl, p, (lvl, "stock")))
            else:
                notes.append("%s: no stock pass" % lvl)
        if args.only in ("removed", "both"):
            found = None
            for sfx in args.suffixes:
                p = os.path.join(args.root, "%s_%s" % (args.run, sfx),
                                 "est_raw_%s.txt" % lvl)
                if os.path.exists(p):
                    found = p
                    break
            if found:
                series.append(("%s threshold removed" % lvl, found,
                               (lvl, "removed")))
            else:
                notes.append("%s: no threshold-removed pass (tried %s)"
                             % (lvl, ", ".join(args.suffixes)))
    for label, path in args.series:
        if os.path.exists(path):
            series.append((label, path, None))
        else:
            notes.append("%s: not found" % path)

    if not series:
        sys.exit("nothing to draw.\n" + "\n".join("  ! " + n for n in notes))

    use_time = args.x == "time"
    fig, ax = plt.subplots(figsize=(11, 5.8), facecolor=PAPER)
    ax.set_facecolor(PAPER)

    got = curve(base, base, use_time) if use_time else None
    if got is not None:
        bx, _e, bt, bxy = got
        for x0, x1 in spans(still_mask(bt, bxy, args.speed), bx):
            if x1 > x0:
                ax.axvspan(x0, x1, color=STILL, alpha=0.55, zorder=0, lw=0)

    rows = []
    for i, (label, path, key) in enumerate(series):
        got = curve(base, path, use_time)
        if got is None:
            notes.append("%s: could not measure" % label)
            continue
        x, err, _t, _b = got
        if key:
            col, dash = series_style(key[0], key[1], args.colour_by)
        else:
            col, dash = SPARE[i % len(SPARE)], "-"
        ax.plot(x, err, lw=2.0, color=col, ls=dash, label=label, zorder=3)
        rows.append((label, x, err, os.path.relpath(path)))

    if not rows:
        sys.exit("nothing measurable.\n" + "\n".join("  ! " + n for n in notes))

    unit = "s" if use_time else "m"
    hdr = ("%-26s %8s %8s %8s %8s %8s %7s  %s"
           % ("series", "at 25%", "at 50%", "at 75%", "final", "peak",
              "at", "file"))
    print("\n%s: separation from the unfiltered pass, by %s travelled\n"
          % (args.run, "time" if use_time else "distance"))
    print(hdr)
    print("-" * len(hdr))
    for label, x, err, path in rows:
        q = [float(np.interp(f * x[-1], x, err)) for f in (0.25, 0.5, 0.75)]
        k = int(np.argmax(err))
        print("%-26s %8.3f %8.3f %8.3f %8.3f %8.3f %6.1f%s  %s"
              % (label, q[0], q[1], q[2], float(err[-1]), float(err[k]),
                 float(x[k]), unit, path))
    for n in notes:
        print("  ! " + n)

    if args.floor > 0:
        ax.axhspan(1e-9, args.floor, color=MUTED, alpha=0.18, zorder=1)
    if not args.linear:
        ax.set_yscale("log")
        top = max(float(np.max(r[2])) for r in rows)
        bot = min(float(np.min(r[2][r[2] > 0])) if np.any(r[2] > 0) else 1e-3
                  for r in rows)
        ax.set_ylim(min(bot * 0.5, args.floor * 0.6), top * 2.4)
    ax.set_xlim(0, max(float(r[1][-1]) for r in rows))
    ax.set_xlabel("time since start [s]" if use_time
                  else "distance travelled along the reference path [m]")
    ax.set_ylabel("separation from unfiltered pass [m]")
    ax.grid(True, which="both", lw=0.7, color=GRID)
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK)
    ax.xaxis.label.set_color(INK)
    ax.yaxis.label.set_color(INK)
    ax.legend(frameon=False, fontsize=13, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.plot, dpi=170)
    print("\nwrote %s" % args.plot)

    print("""
%s
""" % ("""Grey bands are where the rover is stationary, taken from the baseline. True
displacement there is zero, so a curve that climbs inside a band is error you
can state without any reference trajectory at all -- and the height of the
left-hand band should match that run's opening value in parked_bars.py."""
       if use_time else
       """No stationary shading on a distance axis: a parked rover accrues no
distance, so each parked window sits at a single x value. Run again with
--x time to see them, and to check the left-hand flat against parked_bars.py."""))

    print("""

The right-hand endpoint should match brake_effect.py for the same run and
level, because it is the same measurement at the same instant.

""")


if __name__ == "__main__":
    main()
