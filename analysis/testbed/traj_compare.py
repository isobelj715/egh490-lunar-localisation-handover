#!/usr/bin/env python3
"""
Overlay estimated trajectories in the ground plane.

Usage:
    python3 traj_compare.py results/testbed/rectangle/est_raw_L0.txt \
        results/testbed/rectangle/est_raw_L3.txt \
        results/testbed/rectangle_m0f10/est_raw_L3.txt \
        --labels "L0 reference" "L3 brake on" "L3 brake off" \
        --out rectangle_L3_traj.png

The first file is the baseline (the reference path and the frame everything
else is measured against) unless --baseline names one.
"""

import argparse
import os
import sys

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt          # noqa: E402
import numpy as np                       # noqa: E402


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        sys.exit("%s has %d columns, expected at least 4 (t x y z)"
                 % (path, a.shape[1]))
    return a[:, 0], a[:, 1:4]


def plane_of(p):
    """The two axes that are not vertical, chosen by smallest spread."""
    vert = int(np.argmin(np.ptp(p, axis=0)))
    return [i for i in range(3) if i != vert], vert


def resample(t_ref, p_ref, t):
    """Baseline position at the test run's timestamps."""
    return np.column_stack([np.interp(t, t_ref, p_ref[:, k])
                            for k in range(p_ref.shape[1])])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--baseline", help="which file is the reference "
                                       "(default: the first)")
    ap.add_argument("--labels", nargs="*", default=None)
    ap.add_argument("--out", default="traj_compare.png")
    ap.add_argument("--tick", type=float, default=10.0,
                    help="seconds between elapsed-time marks along each path; "
                         "0 to turn them off. Closely spaced marks mean the "
                         "pose is barely moving, widely spaced mean it is "
                         "jumping -- both are visible at a glance.")
    ap.add_argument("--threshold", type=float, default=0.5,
                    help="separation at which a run counts as having lost "
                         "tracking; marked with a star on its path")
    ap.add_argument("--title", default=None)
    ap.add_argument("--frame-tol", type=float, default=0.10,
                    help="how far apart the first poses may be (m) before "
                         "the files are treated as being in different frames")
    args = ap.parse_args()

    files = list(args.files)
    base = args.baseline or files[0]
    if base not in files:
        files.insert(0, base)
    files = [base] + [f for f in files if f != base]

    labels = args.labels or []
    if len(labels) < len(files):
        labels += [os.path.join(
            os.path.basename(os.path.dirname(f)),
            os.path.basename(f).replace("est_raw_", "").replace(".txt", ""))
            for f in files[len(labels):]]

    for f in files:
        if not os.path.exists(f):
            sys.exit("missing: %s" % f)

    t_ref, p_ref = load(base)
    t0 = float(t_ref[0])
    plane, vert = plane_of(p_ref)
    axnames = "xyz"
    t_ref = t_ref - t0

    spread = np.ptp(p_ref, axis=0)
    print("\nbaseline  %s" % base)
    print("  spread   x %.2f  y %.2f  z %.2f m" % tuple(spread))
    print("  vertical '%s' (smallest spread), ground plane %s-%s"
          % (axnames[vert], axnames[plane[0]], axnames[plane[1]]))
    if spread[vert] > 0.5:
        print("  !! %.2f m of vertical motion is a lot for flat sand. If the"
              % spread[vert])
        print("     plane detection is wrong these paths are a projection of")
        print("     the wrong two axes -- check with check_vertical.py first.")

    # Everything is overlaid in ONE frame, so the runs have to be in one frame.
    first_ref = p_ref[0]

    fig, ax = plt.subplots(figsize=(7.5, 7.0))
    colours = ["#444444", "#1b6ca8", "#c1443c", "#2e8b57", "#8a4fb3",
               "#b8860b"]
    rows = []

    for i, (f, lab) in enumerate(zip(files, labels)):
        t, p = load(f)
        t = t - t0
        if np.linalg.norm(p[0] - first_ref) > args.frame_tol:
            sys.exit(
                "%s starts %.2f m from the baseline's first pose.\n"
                "These are not in the same map frame -- overlaying them would "
                "be meaningless.\nLevels of ONE recording share a frame; "
                "different recordings do not."
                % (f, float(np.linalg.norm(p[0] - first_ref))))

        xy = p[:, plane]
        c = colours[i % len(colours)]
        is_base = (f == base)
        ax.plot(xy[:, 0], xy[:, 1], "-", color=c, lw=2.6 if is_base else 1.7,
                alpha=1.0 if is_base else 0.9, zorder=3 if is_base else 2,
                label="%s%s" % (lab, "  (reference)" if is_base else ""))
        ax.plot(xy[0, 0], xy[0, 1], "o", color=c, ms=8, mfc="white", mew=2,
                zorder=4)
        ax.plot(xy[-1, 0], xy[-1, 1], "s", color=c, ms=7, zorder=4)

        # Elapsed-time marks. Spacing along the path is the pose's speed, so
        # bunched marks are a stall and stretched ones a jump.
        if args.tick > 0 and t[-1] > args.tick:
            for s in np.arange(args.tick, t[-1], args.tick):
                k = int(np.searchsorted(t, s))
                if k < len(t):
                    ax.plot(xy[k, 0], xy[k, 1], ".", color=c, ms=5, zorder=4)

        # Where it loses tracking.
        cross_t = None
        if not is_base:
            ref_at = resample(t_ref, p_ref, t)
            sep = np.linalg.norm(p[:, plane] - ref_at[:, plane], axis=1)
            over = np.flatnonzero(sep > args.threshold)
            if len(over):
                k = int(over[0])
                cross_t = float(t[k])
                ax.plot(xy[k, 0], xy[k, 1], "*", color=c, ms=17,
                        mec="black", mew=0.6, zorder=5)
            final = float(sep[-1])
            peak = float(sep.max())
        else:
            final = peak = 0.0

        step = np.linalg.norm(np.diff(p[:, plane], axis=0), axis=1)
        rows.append((lab, len(t), float(t[-1]), float(step.sum()), final,
                     peak, cross_t))

    ax.set_aspect("equal", adjustable="datalim")
    ax.set_xlabel("%s [m]" % axnames[plane[0]])
    ax.set_ylabel("%s [m]" % axnames[plane[1]])
    ax.grid(True, ls=":", alpha=0.45)
    ax.legend(loc="best", fontsize=9, framealpha=0.92)
    ax.set_title(args.title or "trajectories in the ground plane",
                 fontsize=11)
    note = ("open circle = start   filled square = end   "
            "dot = every %g s" % args.tick)
    if args.tick <= 0:
        note = "open circle = start   filled square = end"
    note += "\nstar = first exceeds %.2f m from the reference" % args.threshold
    fig.text(0.5, 0.015, note, ha="center", fontsize=8.5, color="#333333")
    fig.tight_layout(rect=(0, 0.055, 1, 1))
    fig.savefig(args.out, dpi=160)
    print("\nwrote %s" % args.out)

    hdr = "%-30s %6s %8s %9s %8s %8s %9s" % (
        "run", "poses", "dur[s]", "pathlen", "final", "peak", "t>%.2fm"
        % args.threshold)
    print("\n" + hdr)
    print("-" * len(hdr))
    for lab, n, dur, pl, fin, pk, ct in rows:
        print("%-30s %6d %8.1f %9.2f %8.3f %8.3f %9s"
              % (lab[:30], n, dur, pl, fin, pk,
                 "--" if ct is None else "%.1f" % ct))

    print("""
pathlen   distance travelled by the ESTIMATE. The rover's true path length is
          the same in every row, so a departure from the reference's value is
          itself an error: above means the pose is wandering, below means
          motion is being under-estimated.
final     separation from the reference at the last shared timestamp
peak      the worst separation anywhere in the run
t>thr     when it first lost tracking
""")


if __name__ == "__main__":
    main()
