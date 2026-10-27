#!/usr/bin/env python3
"""
What each configuration reports while the rover is PARKED -- both ends.

Usage:
    python3 parked_bars.py --plot parked_drift.png
    python3 parked_bars.py --plot parked_drift.png \
        --pair September results/testbed/final results/testbed/nobrake
    python3 parked_bars.py --window close --plot parked_drift_end.png
"""

import argparse
import glob
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

STYLE = [
    ("L0 control",           LEVEL_HUE["L0"]),
    ("L2 stock",             LEVEL_PALE["L2"]),
    ("L2 threshold removed", LEVEL_HUE["L2"]),
    ("L3 stock",             LEVEL_PALE["L3"]),
    ("L3 threshold removed", LEVEL_HUE["L3"]),
]


def load(path, t0=None):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        return None
    t0 = a[0, 0] if t0 is None else t0
    return a[:, 0] - t0, a[:, 1:4]


def plane_of(p):
    return [i for i in range(3) if i != int(np.argmin(np.ptp(p, axis=0)))]


def speed_of(t, xy):
    step = np.r_[0.0, np.linalg.norm(np.diff(xy, axis=0), axis=1)]
    dt = np.r_[1e-6, np.diff(t)]
    return step / np.maximum(dt, 1e-6)


def still_windows(t, xy, thr, margin, min_park):
    v = speed_of(t, xy)
    moving = np.flatnonzero(v > thr)
    diag = {"dur": float(t[-1] - t[0]),
            "v_med": float(np.median(v)),
            "v_lo": float(np.percentile(v, 10)),
            "moving_frac": float(len(moving)) / max(len(v), 1)}
    if not len(moving):
        return (float(t[0]), float(t[-1])), None, diag

    first = None
    for i in moving:                       # ignore a single noisy sample
        if i + 3 < len(v) and np.all(v[i:i + 3] > thr * 0.5):
            first = float(t[i])
            break
    if first is None:
        first = float(t[moving[0]])

    last = None
    for i in moving[::-1]:
        if i - 3 >= 0 and np.all(v[i - 3:i] > thr * 0.5):
            last = float(t[i])
            break
    if last is None:
        last = float(t[moving[-1]])

    opening = (float(t[0]), max(first - margin, float(t[0])))
    closing = (min(last + margin, float(t[-1])), float(t[-1]))
    diag["first"] = first
    diag["last"] = last
    if opening[1] - opening[0] < min_park:
        opening = None
    if closing[1] - closing[0] < min_park:
        closing = None
    return opening, closing, diag


def measure(t, xy, window, frac):
    """Displacement from the first pose in the window, read `frac` through."""
    if window is None:
        return None
    lo, hi = window
    sel = (t >= lo) & (t <= hi)
    if sel.sum() < 5:
        return None
    ts, p = t[sel], xy[sel]
    disp = np.linalg.norm(p - p[0], axis=1)
    k = int(np.argmax(disp))
    return {
        "span": hi - lo,
        "at": float(np.interp(lo + frac * (hi - lo), ts, disp)),
        "end": float(disp[-1]),
        "max": float(disp[k]),
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", help="stock run directories")
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--levels", nargs="+", default=["L2", "L3"])
    ap.add_argument("--suffixes", nargs="+",
                    default=["m0f10", "m10f10", "m0f0", "nobrake"])
    ap.add_argument("--pair", nargs=3, action="append", default=[],
                    metavar=("NAME", "STOCKDIR", "CORRECTEDDIR"),
                    help="a run whose corrected build does not follow the "
                         "<run>_<suffix> convention; repeatable")
    ap.add_argument("--at", type=float, default=0.75,
                    help="fraction through the window to report (0-1)")
    ap.add_argument("--window", choices=["open", "close"], default="open",
                    help="which stationary window the FIGURE draws; both are "
                         "printed either way")
    ap.add_argument("--speed", type=float, default=0.05,
                    help="m/s below which the rover counts as stationary")
    ap.add_argument("--min-park", type=float, default=4.0)
    ap.add_argument("--margin", type=float, default=1.5)
    ap.add_argument("--floor", type=float, default=0.012)
    ap.add_argument("--plot", default="parked_drift.png")
    args = ap.parse_args()

    explicit = {n: (s, c) for n, s, c in args.pair}
    named = {d.rstrip("/") for p in args.pair for d in p[1:]}
    runs = args.runs or sorted(
        d.rstrip("/") for d in glob.glob(args.root + "/*/")
        if "_" not in os.path.basename(d.rstrip("/"))
        and d.rstrip("/") not in named)
    order = list(explicit) + [os.path.basename(os.path.normpath(d))
                              for d in runs]

    rows, notes = [], []
    for name in order:
        if name in explicit:
            stock_dir, fixed_dir = explicit[name]
        else:
            stock_dir = os.path.join(args.root, name)
            fixed_dir = None

        base = os.path.join(stock_dir, "est_raw_L0.txt")
        if not os.path.exists(base):
            notes.append("%s: no L0 pass, so no stationary window and no "
                         "control" % name)
            continue

        raw = np.loadtxt(base, ndmin=2)
        origin = float(raw[0, 0])
        t, pos = load(base, origin)
        xy = pos[:, plane_of(pos)]
        opening, closing, d = still_windows(t, xy, args.speed, args.margin,
                                            args.min_park)
        if opening is None and closing is None:
            notes.append(
                "%s: no stationary stretch of %.1f s at either end -- "
                "%.0f s long, first motion %.1f s, last %.1f s, median speed "
                "%.3f m/s, %.0f%% of samples above --speed %.2f. If the "
                "median is near --speed the window detector is seeing noise, "
                "not motion: raise --speed. If first motion is at 0 s and "
                "last is at the end, the recording was trimmed to the drive "
                "and has no parked window to measure."
                % (name, args.min_park, d["dur"], d.get("first", 0.0),
                   d.get("last", d["dur"]), d["v_med"],
                   100 * d["moving_frac"], args.speed))
            continue

        conds = [("L0 control", base)]
        for lvl in args.levels:
            conds.append(("%s stock" % lvl,
                          os.path.join(stock_dir, "est_raw_%s.txt" % lvl)))
            if fixed_dir is not None:
                conds.append(("%s threshold removed" % lvl,
                              os.path.join(fixed_dir,
                                           "est_raw_%s.txt" % lvl)))
            else:
                for sfx in args.suffixes:
                    p = os.path.join(args.root, "%s_%s" % (name, sfx),
                                     "est_raw_%s.txt" % lvl)
                    if os.path.exists(p):
                        conds.append(("%s threshold removed" % lvl, p))
                        break

        for label, path in conds:
            if not os.path.exists(path):
                continue
            got = load(path, origin)
            if got is None:
                continue
            tc, pc = got
            xyc = pc[:, plane_of(pc)]
            rows.append((name, label, os.path.relpath(path),
                         measure(tc, xyc, opening, args.at),
                         measure(tc, xyc, closing, args.at)))

    if not rows:
        sys.exit("nothing measured.\n" + "\n".join(notes))

    hdr = ("%-11s %-21s %6s %8s %8s %6s %8s %8s"
           % ("run", "condition", "open", "@%d%%" % (args.at * 100), "max",
              "close", "@%d%%" % (args.at * 100), "max"))
    print("\nreported displacement while parked [m] -- true value is zero\n")
    print(hdr)
    print("-" * len(hdr))
    last = None
    for name, label, _p, o, c in rows:
        if last is not None and name != last:
            print("")
        last = name
        f = lambda d, k: ("%8.3f" % d[k]) if d else "       -"
        print("%-11s %-21s %5s %s %s %5s %s %s"
              % (name, label,
                 ("%.0fs" % o["span"]) if o else "-", f(o, "at"), f(o, "max"),
                 ("%.0fs" % c["span"]) if c else "-", f(c, "at"), f(c, "max")))
    for n in notes:
        print("  ! " + n)

    groups = []
    for name in order:
        vals = {}
        for rn, lab, _p, o, c in rows:
            if rn == name:
                d = o if args.window == "open" else c
                if d:
                    vals[lab] = d["at"]
        if vals:
            groups.append((name, vals))
    if not groups:
        sys.exit("no %sing-window values to plot" % args.window)

    fig, ax = plt.subplots(figsize=(12, 5.6), facecolor=PAPER)
    ax.set_facecolor(PAPER)
    present = [s for s in STYLE if any(s[0] in g[1] for g in groups)]
    n = len(present)
    w = 0.8 / n
    x = np.arange(len(groups))
    for i, (lab, col) in enumerate(present):
        vals = [g[1].get(lab, np.nan) for g in groups]
        off = (i - (n - 1) / 2) * w
        ax.bar(x + off, vals, w * 0.92, color=col, label=lab)
        for xi, v in zip(x + off, vals):
            if not np.isnan(v):
                ax.text(xi, v * 1.16, ("%.3f" % v) if v < 1 else ("%.2f" % v),
                        ha="center", fontsize=11, color=INK, rotation=90)

    ax.axhspan(1e-9, args.floor, color=MUTED, alpha=0.2, zorder=0)
    ax.set_yscale("log")
    flat = [v for _g, d in groups for v in d.values() if v > 0]
    ax.set_ylim(min(min(flat) * 0.4, args.floor * 0.6), max(flat) * 7)
    ax.set_xticks(x, [g[0] for g in groups])
    ax.set_ylabel("reported displacement while parked [m]")
    ax.set_title("%sing stationary window -- the rover has not moved"
                 % ("Open" if args.window == "open" else "Clos"),
                 color=INK, fontsize=15, loc="left")
    ax.grid(True, which="both", lw=0.7, color=GRID, axis="y")
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK)
    ax.yaxis.label.set_color(INK)
    ax.legend(frameon=False, ncol=n, fontsize=12, loc="upper left")
    fig.tight_layout()
    fig.savefig(args.plot, dpi=170)
    print("\nwrote %s" % args.plot)

    print("""
open / close      length of each stationary window, found from that run's own
                  unfiltered L0 pass and backed off %.1f s from the motion
@%-3d              displacement that far through the window -- not the max,
                  which catches the rover pulling away
max               largest displacement anywhere in the window

The L0 row is the control: if it reads millimetres, the window really was
still, and every larger number in that run is error owing nothing to a
reference trajectory.
""" % (
        args.margin, int(args.at * 100)))


if __name__ == "__main__":
    main()
