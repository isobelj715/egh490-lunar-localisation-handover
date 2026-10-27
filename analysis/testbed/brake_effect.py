#!/usr/bin/env python3
"""
Error at the end of the run, stock against threshold-removed, every run.

Usage:
    python3 brake_effect.py --plot brake_effect_L2.png
    python3 brake_effect.py --level L3 --plot brake_effect_L3.png
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
STOCK = "#C4521F"
FIXED = "#1B6CA8"


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        return None
    return a[:, 0], a[:, 1:4]


def plane_of(p):
    v = int(np.argmin(np.ptp(p, axis=0)))
    return [i for i in range(3) if i != v]


def final_error(base, est):
    """Separation at the last shared timestamp, in the ground plane."""
    b, e = load(base), load(est)
    if b is None or e is None:
        return None
    tb, pb = b
    te, pe = e
    t_end = min(tb[-1], te[-1])
    t_beg = max(tb[0], te[0])
    if t_end <= t_beg:
        return None
    plane = plane_of(pb)
    at = lambda t, p: np.array(
        [np.interp(t_end, t, p[:, k]) for k in plane])
    return float(np.linalg.norm(at(tb, pb) - at(te, pe)))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("runs", nargs="*", help="stock run directories")
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--level", default="L2")
    ap.add_argument("--suffixes", nargs="+",
                    default=["m0f10", "m10f10", "m0f0", "nobrake"],
                    help="tried in order to find each run's corrected pass")
    ap.add_argument("--pair", nargs=3, action="append", default=[],
                    metavar=("NAME", "STOCKDIR", "CORRECTEDDIR"),
                    help="name a run whose corrected build does not follow "
                         "the <run>_<suffix> convention; repeatable")
    ap.add_argument("--floor", type=float, default=0.012)
    ap.add_argument("--plot", default="brake_effect.png")
    args = ap.parse_args()

    rows, notes = [], []

    def as_file(p):
        return p if p.endswith(".txt") else os.path.join(
            p, "est_raw_%s.txt" % args.level)

    for name, sd, fd in args.pair:
        stock, fixed = as_file(sd), as_file(fd)
        base = os.path.join(os.path.dirname(stock), "est_raw_L0.txt")
        if not all(os.path.exists(p) for p in (base, stock, fixed)):
            notes.append("%s: a file in the given pair is missing" % name)
            continue
        a, b = final_error(base, stock), final_error(base, fixed)
        if a is None or b is None:
            notes.append("%s: could not compute an error" % name)
            continue
        rows.append((name, a, b,
                     os.path.relpath(fixed) if fixed.endswith(".txt")
                     else os.path.basename(os.path.normpath(fd))))

    named = {p[1].rstrip("/") for p in args.pair} | {
        p[2].rstrip("/") for p in args.pair}
    runs = args.runs or sorted(
        d.rstrip("/") for d in glob.glob(args.root + "/*/")
        if "_" not in os.path.basename(d.rstrip("/"))
        and d.rstrip("/") not in named)
    if not runs and not rows:
        sys.exit("no run directories under %s" % args.root)

    have = {r[0] for r in rows}
    for d in runs:
        name = os.path.basename(os.path.normpath(d))
        if name in have:          # already supplied by an explicit --pair
            continue
        base = os.path.join(d, "est_raw_L0.txt")
        stock = os.path.join(d, "est_raw_%s.txt" % args.level)
        if not (os.path.exists(base) and os.path.exists(stock)):
            notes.append("%s: no baseline or no stock %s" % (name, args.level))
            continue
        fixed, used = None, None
        for sfx in args.suffixes:
            p = os.path.join(args.root, "%s_%s" % (name, sfx),
                             "est_raw_%s.txt" % args.level)
            if os.path.exists(p):
                fixed, used = p, sfx
                break
        if fixed is None:
            notes.append("%s: no corrected %s found (tried %s)"
                         % (name, args.level, ", ".join(args.suffixes)))
            continue
        a = final_error(base, stock)
        b = final_error(base, fixed)
        if a is None or b is None:
            notes.append("%s: could not compute an error" % name)
            continue
        rows.append((name, a, b, used))

    if not rows:
        sys.exit("nothing to plot.\n" + "\n".join(notes))

    hdr = ("%-14s %10s %12s %9s   %s"
           % ("run", "stock", "corrected", "factor", "corrected dir"))
    print("\nerror at end of run, level %s\n" % args.level)
    print(hdr)
    print("-" * len(hdr))
    for name, a, b, used in rows:
        print("%-14s %9.3f m %11.3f m %8.0fx   %s"
              % (name, a, b, a / max(b, 1e-9),
                 used if "/" in used or used.startswith(name)
                 else "%s_%s" % (name, used)))
    for n in notes:
        print("  ! " + n)

    fig, ax = plt.subplots(figsize=(11, 5.8), facecolor=PAPER)
    ax.set_facecolor(PAPER)
    x = np.arange(len(rows))
    w = 0.36
    ax.bar(x - w / 2, [r[1] for r in rows], w, color=STOCK,
           label="stock (eignThre = 100)")
    ax.bar(x + w / 2, [r[2] for r in rows], w, color=FIXED,
           label="threshold removed")
    for i, (_n, a, b, _u) in enumerate(rows):
        ax.text(i - w / 2, a * 1.13, "%.2f m" % a, ha="center", fontsize=14,
                color=INK)
        ax.text(i + w / 2, b * 1.13, "%.3f m" % b, ha="center", fontsize=14,
                color=INK)

    ax.axhspan(0, args.floor, color=MUTED, alpha=0.22, zorder=0)
    ax.text(-0.45, args.floor * 1.15, "repeat-to-repeat noise floor, %g m"
            % args.floor, ha="left", va="bottom", fontsize=13)
    ax.set_yscale("log")
    top = max(max(r[1], r[2]) for r in rows)
    bot = min(min(r[1], r[2]) for r in rows)
    ax.set_ylim(min(bot * 0.45, args.floor * 0.75), top * 3.2)
    ax.set_xticks(x, ["%s\n%.0f× better" % (r[0], r[1] / max(r[2], 1e-9))
                      if r[1] > r[2] else "%s\nno improvement" % r[0]
                      for r in rows])
    ax.set_ylabel("error at end of run [m]")
    ax.grid(True, which="both", lw=0.7, color=GRID, axis="y")
    ax.set_axisbelow(True)
    for s in ("top", "right"):
        ax.spines[s].set_visible(False)
    ax.spines["left"].set_color(GRID)
    ax.spines["bottom"].set_color(GRID)
    ax.tick_params(colors=INK)
    ax.yaxis.label.set_color(INK)
    ax.legend(frameon=False, loc="upper left", ncol=2, fontsize=14)
    fig.tight_layout()
    fig.savefig(args.plot, dpi=170)
    print("\nwrote %s" % args.plot)

    print("""
Measured against each run's own unfiltered pass, so these are lower bounds on
absolute error -- the bound applies equally to both bars, which is what the
comparison needs.
""")


if __name__ == "__main__":
    main()
