#!/usr/bin/env python3
"""
Pose observability against structure level, for every run.

WHAT IT PLOTS
The median lambda_min of the map-refinement stage at each ablation level,
one line per recording, read from each run's own structure.csv. Nothing is
hard-coded: add a run and it appears.

Usage:
    python3 lambda_ladder.py --plot lambda_ladder.png
    python3 lambda_ladder.py results/testbed/rectangle results/testbed/zigzag
    python3 lambda_ladder.py --threshold 0 --plot structure_only.png
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
WARN = "#C4521F"
SERIES = ["#1B6CA8", "#C4521F", "#2E8B57", "#8A4FB3", "#B8860B", "#4C6EF5"]
MARKS = ["o", "s", "^", "D", "v", "P"]

LEVELS = ["L0", "L1", "L2", "L3"]
MEANS = {"L0": "everything", "L1": "no building",
         "L2": "no outer walls", "L3": "sand only"}


def lambda_of(path, stat):
    """Median (or mean) map lambda_min from one level's structure.csv."""
    with open(path) as fh:
        cols = [c.strip() for c in fh.readline().strip().split(",")]
    j = next((i for i, c in enumerate(cols) if c == "map_full_lambda_min"),
             None)
    if j is None:
        j = next((i for i, c in enumerate(cols) if "lambda_min" in c), None)
    if j is None:
        return None, 0
    a = np.genfromtxt(path, delimiter=",", skip_header=1)
    if a.ndim == 1:
        a = a[None, :]
    v = a[:, j]
    v = v[np.isfinite(v)]
    if not len(v):
        return None, 0
    return float(np.median(v) if stat == "median" else np.mean(v)), len(v)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*",
                    help="result directories; default is every stock run "
                         "under results/testbed")
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--labels", nargs="*", default=[],
                    help="display names, in the order the runs were given. "
                         "Use it when a directory name is not the run name")
    ap.add_argument("--stat", choices=["median", "mean"], default="median")
    ap.add_argument("--threshold", type=float, default=100.0,
                    help="eignThre line; 0 leaves it off")
    ap.add_argument("--plot", default="lambda_ladder.png")
    args = ap.parse_args()

    runs = args.runs
    if not runs:
        # Stock runs only. 
        runs = sorted(d.rstrip("/") for d in glob.glob(args.root + "/*/")
                      if "_" not in os.path.basename(d.rstrip("/")))
    if not runs:
        sys.exit("no run directories found under %s" % args.root)

    if args.labels and len(args.labels) != len(runs):
        sys.exit("--labels has %d names for %d runs"
                 % (len(args.labels), len(runs)))

    table, skipped = {}, []
    for i, d in enumerate(runs):
        name = (args.labels[i] if args.labels
                else os.path.basename(os.path.normpath(d)))
        row = {}
        for lv in LEVELS:
            p = os.path.join(d, "logs", lv, "structure.csv")
            if not os.path.exists(p):
                continue
            val, n = lambda_of(p, args.stat)
            if val is not None:
                row[lv] = (val, n)
        if row:
            table[name] = row
        else:
            skipped.append(name)

    if not table:
        sys.exit("no structure.csv files found. Expected "
                 "<run>/logs/<L0..L3>/structure.csv")

    hdr = "%-14s" % "run" + "".join("%12s" % lv for lv in LEVELS) + "   L0/L3"
    print("\n%s lambda_min of the map stage\n" % args.stat)
    print(hdr)
    print("-" * len(hdr))
    for name, row in table.items():
        line = "%-14s" % name[:14]
        for lv in LEVELS:
            line += "%12s" % ("%.1f" % row[lv][0] if lv in row else "-")
        if "L0" in row and "L3" in row:
            line += "   %5.0fx" % (row["L0"][0] / max(row["L3"][0], 1e-9))
        print(line)
    if skipped:
        print("\nno structure.csv in: %s" % ", ".join(skipped))

    fig, ax = plt.subplots(figsize=(11, 5.8), facecolor=PAPER)
    ax.set_facecolor(PAPER)
    xs = np.arange(len(LEVELS))

    if args.threshold > 0:
        ax.axhspan(1e-3, args.threshold, color=WARN, alpha=0.10, zorder=0)
        ax.axhline(args.threshold, color=INK, lw=2.0, ls="--", zorder=2)
        ax.text(len(LEVELS) - 0.55, args.threshold * 1.18,
                "hard-coded eignThre = %g" % args.threshold,
                ha="right", fontsize=14, color=INK)

    lo = 1e9
    for i, (name, row) in enumerate(table.items()):
        kx = [k for k, lv in enumerate(LEVELS) if lv in row]
        ky = [row[LEVELS[k]][0] for k in kx]
        lo = min(lo, min(ky))
        ax.plot(kx, ky, MARKS[i % len(MARKS)] + "-",
                color=SERIES[i % len(SERIES)], lw=2.2, ms=9, label=name)

    ax.set_yscale("log")
    ax.set_xlim(-0.35, len(LEVELS) - 0.65)
    ax.set_ylim(max(lo * 0.45, 1e-3), 1500)
    ax.set_xticks(xs, ["%s\n%s" % (lv, MEANS[lv]) for lv in LEVELS])
    ax.set_ylabel("$\\lambda_{min}$  (how well the scene pins the pose)")
    ax.grid(True, which="both", lw=0.7, color=GRID, axis="y")
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.legend(frameon=False, loc="lower left", fontsize=14)
    for lab in ax.get_xticklabels() + ax.get_yticklabels():
        lab.set_color(INK)
    ax.tick_params(colors=INK)
    ax.yaxis.label.set_color(INK)
    fig.tight_layout()
    fig.savefig(args.plot, dpi=170)
    print("\nwrote %s" % args.plot)

    print("""
Each point is the %s over every scan in that level of that run, so a line is
one recording and the x-axis is how much of the scene was deleted before it
was replayed. The same physical drive underlies all four points on a line.
""" % args.stat)


if __name__ == "__main__":
    main()
