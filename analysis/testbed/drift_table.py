#!/usr/bin/env python3
"""
Compare trajectories on a COMMON time window, and report enough of the
distribution to see a late blow-up.

Usage:
    python3 drift_table.py --baseline results/testbed/levels/est_raw_L0.txt \
        results/testbed/levels/est_raw_L*.txt \
        results/testbed/refilter/est_raw_FIX*.txt

    # force a window, e.g. to compare against previously quoted full-run numbers
    python3 drift_table.py --baseline base.txt others*.txt --window 0 99
"""

import argparse
import os
import sys

import numpy as np

np.seterr(all="ignore")


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        sys.exit("%s: expected at least 4 columns (t x y z ...)" % path)
    label = path.replace("est_raw_", "").replace(".txt", "")
    for cut in ("results/testbed/", "results/"):
        if cut in label:
            label = label.split(cut)[-1]
    return label, a[:, 0], a[:, 1:4]


# Which two of the three pose columns are the GROUND PLANE. Set from the data
# in main(); the default is only a fallback.
PLANE = (0, 2)
AXIS_NAMES = "xyz"


def detect_plane(p):
    """(ground-plane column pair, vertical column) from the spreads."""
    spread = np.ptp(p, axis=0)
    vert = int(np.argmin(spread))
    return tuple(i for i in range(3) if i != vert), vert, spread


def horiz(p):
    return p[:, list(PLANE)]


def resample(t, p, grid):
    return np.column_stack([np.interp(grid, t, p[:, k]) for k in range(3)])


def path_length(t, p, t0, t1, full3d):
    m = (t >= t0) & (t <= t1)
    q = p[m] if full3d else horiz(p[m])
    if len(q) < 2:
        return float("nan")
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--baseline", required=True,
                    help="trajectory the others are compared against")
    ap.add_argument("--window", nargs=2, type=float, default=None,
                    metavar=("T0", "T1"),
                    help="seconds relative to the baseline's first pose. "
                         "Default: the interval every run covers.")
    ap.add_argument("--threshold", type=float, default=0.5,
                    help="report when divergence first exceeds this (m)")
    ap.add_argument("--plane", choices=["auto", "xy", "xz", "yz"],
                    default="auto",
                    help="which columns are the ground plane. 'auto' takes the "
                         "two with the largest spread, since a rover on flat "
                         "sand barely moves vertically.")
    ap.add_argument("--cvd-palette", action="store_true",
                    help="colourblind-safe series colours instead of the "
                         "matplotlib defaults. Default red and green are the "
                         "classic confusion pair, and here they carry the two "
                         "failing levels.")
    ap.add_argument("--full-3d", action="store_true")
    ap.add_argument("--grid-hz", type=float, default=5.0)
    ap.add_argument("--plot", default=None,
                    help="write a two-panel figure: trajectories, and "
                         "divergence over time")
    ap.add_argument("--log-y", action="store_true",
                    help="log scale on the divergence panel. ")
    ap.add_argument("--noise-floor", type=float, default=0.012,
                    help="repeat-to-repeat median separation, shaded on the "
                         "divergence panel so the reader can see what is "
                         "resolvable (default 0.012 m, measured at 0.3x)")
    args = ap.parse_args()

    paths = [args.baseline] + [f for f in args.files
                               if os.path.abspath(f) != os.path.abspath(args.baseline)]
    runs = []
    for f in paths:
        runs.append(load(f))

    base_t0 = runs[0][1][0]

    # ---- which plane is the ground? -------------------------------------
    global PLANE
    auto_plane, vert, spread = detect_plane(runs[0][2])
    if args.plane == "auto":
        PLANE = auto_plane
    else:
        PLANE = tuple(AXIS_NAMES.index(c) for c in args.plane)
    print("")
    print("POSE AXES in %s: spread x %.2f m, y %.2f m, z %.2f m"
          % (runs[0][0], spread[0], spread[1], spread[2]))
    print("  vertical axis looks like '%s' (smallest spread), so the ground "
          "plane is %s-%s%s"
          % (AXIS_NAMES[vert], AXIS_NAMES[auto_plane[0]],
             AXIS_NAMES[auto_plane[1]],
             "" if args.plane == "auto" else "  [OVERRIDDEN to %s]" % args.plane))
    if spread[vert] > 1.0:
        print("  WARNING: the smallest spread is %.2f m, which is a lot of "
              "vertical" % spread[vert])
        print("  motion for a rover on flat sand. Either the run has real "
              "height drift")
        print("  or the plane detection is being fooled -- check before "
              "trusting these numbers.")
    print("  Distances below use the %s-%s plane only."
          % (AXIS_NAMES[PLANE[0]], AXIS_NAMES[PLANE[1]]))

    # ---- coverage -------------------------------------------------------
    print("")
    print("COVERAGE  (seconds relative to the baseline's first pose)")
    print("%-34s %6s %8s %8s" % ("run", "poses", "start", "end"))
    print("-" * 60)
    for lab, t, p in runs:
        print("%-34s %6d %8.1f %8.1f"
              % (lab, len(t), t[0] - base_t0, t[-1] - base_t0))

    latest_start = max(t[0] for _, t, _ in runs) - base_t0
    earliest_end = min(t[-1] for _, t, _ in runs) - base_t0

    if args.window:
        w0, w1 = args.window
        if w0 < latest_start - 1e-6 or w1 > earliest_end + 1e-6:
            print("\nWARNING: requested window [%.1f, %.1f] is not covered by every"
                  % (w0, w1))
            print("run (common interval is [%.1f, %.1f]). Runs are extrapolated"
                  % (latest_start, earliest_end))
            print("outside their own data, which is not meaningful -- treat any")
            print("row whose end is below %.1f s as unusable." % w1)
    else:
        w0, w1 = latest_start, earliest_end

    print("\nwindow used: [%.1f, %.1f] s  (%.1f s)" % (w0, w1, w1 - w0))
    short = [lab for lab, t, _ in runs if (t[-1] - base_t0) < w1 - 1.0]
    if short and not args.window:
        print("set by the earliest-terminating run: %s" % ", ".join(short))
        print("A run that stops early is itself a result -- note it, do not hide it.")

    n = max(int((w1 - w0) * args.grid_hz), 8)
    grid = base_t0 + np.linspace(w0, w1, n)

    bl = runs[0]
    B = resample(bl[1], bl[2], grid)

    # ---- table ----------------------------------------------------------
    print("")
    hdr = ("%-34s %8s %8s %8s %8s %9s"
           % ("run", "median", "p95", "final", "pathlen", "t>%.1fm" % args.threshold))
    print(hdr)
    print("-" * len(hdr))
    for lab, t, p in runs:
        P = resample(t, p, grid)
        d = P - B
        dist = (np.linalg.norm(d, axis=1) if args.full_3d
                else np.linalg.norm(d[:, list(PLANE)], axis=1))
        L = path_length(t, p, grid[0], grid[-1], args.full_3d)
        over = np.flatnonzero(dist > args.threshold)
        tcross = ("%9.1f" % (grid[over[0]] - base_t0)) if len(over) else "%9s" % "--"
        flag = "  *" if (t[-1] - base_t0) < w1 - 1.0 else ""
        print("%-34s %8.3f %8.3f %8.3f %8.2f %s%s"
              % (lab, np.median(dist), np.percentile(dist, 95), dist[-1], L,
                 tcross, flag))

    print("""
median   typical separation over the window
p95      separation on the worst 5% of the window -- catches a late blow-up
final    separation at the end of the window; the number a median can hide
pathlen  distance travelled IN THE WINDOW. The true value is the same for every
         run, so a departure from the baseline's value is itself an error signal:
         above it means the pose is wandering, below it means motion is being
         under-estimated.
t>thr    when divergence first exceeds --threshold. A late crossing with a small
         median is a run that tracked and then broke, which is a different
         failure from one that never tracked.
*        run terminates before the window ends; its rows are extrapolated.

Drift is measured against the baseline, not ground truth, so the baseline's own
error is common to every row and these are LOWER BOUNDS on absolute error.""")

    if args.plot:
        plot(args, runs, grid, base_t0, B, w0, w1)

SERIES_DEFAULT = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728",
                  "#9467bd", "#8c564b"]
SERIES_CVD = ["#0072B2", "#D55E00", "#009E73", "#E69F00", "#CC79A7", "#56B4E9"]
INK, INK_MUTED, GRID = "#1a1a1a", "#6b6b6b", "#d8d8d4"
SURFACE = "#ffffff"


def plot(args, runs, grid, base_t0, B, w0, w1):
    """Two panels: the paths themselves, and how far each is from the baseline.
    """
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt

    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(13, 5.6),
                                   facecolor=SURFACE)
    for ax in (ax1, ax2):
        ax.set_facecolor(SURFACE)
        ax.grid(True, color=GRID, lw=0.7, zorder=0)
        ax.set_axisbelow(True)
        ax.tick_params(labelsize=9)

    SERIES = SERIES_CVD if args.cvd_palette else SERIES_DEFAULT
    ax_u, ax_v = AXIS_NAMES[PLANE[0]], AXIS_NAMES[PLANE[1]]

    # ---- panel 1: the trajectories -----------------------------------
    for k, (lab, t, p) in enumerate(runs):
        m = (t >= grid[0]) & (t <= grid[-1])
        uv = horiz(p[m])
        if len(uv) < 2:
            continue
        col = SERIES[k % len(SERIES)]
        ax1.plot(uv[:, 0], uv[:, 1], lw=1.2, color=col, label=lab,
                 zorder=3 + k)
        ax1.plot(uv[0, 0], uv[0, 1], "o", ms=6, mfc="none", mec=col, mew=1.4,
                 zorder=20)
        ax1.plot(uv[-1, 0], uv[-1, 1], "x", ms=7, color=col, mew=1.8,
                 zorder=20)
    ax1.set_aspect("equal")
    ax1.set_xlabel("%s [m]" % ax_u, fontsize=10)
    ax1.set_ylabel("%s [m]" % ax_v, fontsize=10)
    ax1.set_title("trajectories (o start, x end)", fontsize=11)
    ax1.legend(fontsize=9, loc="upper left", framealpha=0.9)

    # ---- panel 2: divergence from the baseline ------------------------
    tt = grid - base_t0
    if args.noise_floor > 0:
        ax2.axhspan(0, args.noise_floor, color=GRID, alpha=0.9, zorder=1)
    ends = []
    for k, (lab, t, p) in enumerate(runs):
        if k == 0:
            continue
        P = resample(t, p, grid)
        d = P - B
        dist = (np.linalg.norm(d, axis=1) if args.full_3d
                else np.linalg.norm(d[:, list(PLANE)], axis=1))
        col = SERIES[k % len(SERIES)]
        ax2.plot(tt, dist, lw=1.2, color=col, label=lab, zorder=3 + k)
        ends.append([float(dist[-1]), lab, col])
    ax2.set_xlabel("t [s]", fontsize=10)
    ax2.set_ylabel("[m]", fontsize=10)
    ax2.set_title("divergence from %s" % runs[0][0], fontsize=11)
    ax2.set_xlim(tt[0], tt[-1] + (tt[-1] - tt[0]) * 0.10)
    if args.log_y:
        ax2.set_yscale("log")
        ax2.set_ylim(bottom=max(args.noise_floor, 1e-4) / 4.0)
    else:
        ax2.set_ylim(bottom=0)
    ax2.legend(fontsize=9, loc="upper left", framealpha=0.9)
    # End-of-line labels, nudged apart. Two runs can finish at nearly the same
    # error and their labels would then overprint, which is worse than no
    # label at all.
    if len(runs) - 1 <= 4 and ends:
        lo, hi = ax2.get_ylim()
        gap = (hi - lo) * 0.045
        ends.sort(key=lambda e: e[0])
        for i in range(1, len(ends)):
            if ends[i][0] - ends[i - 1][0] < gap:
                ends[i][0] = ends[i - 1][0] + gap
        for y, lab, col in ends:
            ax2.annotate(lab, xy=(tt[-1], min(y, hi)), xytext=(6, 0),
                         textcoords="offset points", fontsize=9, color=INK,
                         va="center", zorder=20, annotation_clip=False)

    # The shaded floor needs naming, placed clear of the traces rather than on
    # top of the series that hugs it.
    if args.noise_floor > 0:
        ax2.annotate("repeat-to-repeat noise, %.3f m — anything inside this "
                     "band is the pipeline, not the manipulation"
                     % args.noise_floor,
                     xy=(0.5, -0.155), xycoords="axes fraction",
                     ha="center", fontsize=8.5, color=INK_MUTED)

    fig.tight_layout()
    fig.savefig(args.plot, dpi=140, facecolor=SURFACE, bbox_inches="tight")
    print("\nwrote %s" % args.plot)


if __name__ == "__main__":
    main()
