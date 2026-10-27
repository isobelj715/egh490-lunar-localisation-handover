#!/usr/bin/env python3
"""
Is the vertical excursion in a trajectory real terrain, or vertical drift?

Real terrain is a property of the place: the height reported on the second visit to a place must
match the height reported on the first. Drift is a property of elapsed time:
the second visit comes back at a different height. 

Usage:
    python3 check_vertical.py results/testbed/rectangle/est_raw_L*.txt
    python3 check_vertical.py results/testbed/rectangle/est_raw_L*.txt \
            --plot rectangle_vertical.png
    python3 check_vertical.py ... --vert y        # override axis detection
"""

import argparse
import os
import sys

import numpy as np

SERIES = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd", "#8c564b"]
GRID, INK, INK_MUTED = "#d8d8d4", "#1a1a1a", "#6b6b6b"
AXES = "xyz"


def load(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        sys.exit("%s: expected at least 4 columns (t x y z ...)" % path)
    name = os.path.basename(path).replace("est_raw_", "").replace(".txt", "")
    return name, a[:, 0] - a[0, 0], a[:, 1:4]


def pick_vertical(p):
    """Smallest spread is height. For LeGO-LOAM's /aft_mapped_to_init this is
    y, because the published odometry keeps LOAM's internal frame where y is
    up -- so an x-z ground plane is expected, not a detection failure."""
    sp = np.ptp(p, axis=0)
    return int(np.argmin(sp)), sp


def closest_revisit(xy, t, min_gap):
    """Closest approach between two poses more than min_gap seconds apart."""
    best = (np.inf, -1, -1)
    for i in range(len(t)):
        far = np.flatnonzero(t > t[i] + min_gap)
        if not far.size:
            continue
        d = np.linalg.norm(xy[far] - xy[i], axis=1)
        k = int(np.argmin(d))
        if d[k] < best[0]:
            best = (float(d[k]), i, int(far[k]))
    return best


def overlap_mismatch(xy, z, t, min_gap, tol):
    """Height disagreement over EVERY pair of poses that pass within tol of
    each other, separated by more than min_gap. Terrain is a function of
    position, so these must all agree; drift makes the mismatch grow with the
    time between the two visits."""
    dz, dt = [], []
    for i in range(len(t)):
        far = np.flatnonzero(t > t[i] + min_gap)
        if not far.size:
            continue
        d = np.linalg.norm(xy[far] - xy[i], axis=1)
        hit = far[d < tol]
        for j in hit:
            dz.append(abs(z[j] - z[i]))
            dt.append(t[j] - t[i])
    return np.array(dz), np.array(dt)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("files", nargs="+")
    ap.add_argument("--vert", default=None, choices=list(AXES),
                    help="force the vertical axis instead of detecting it")
    ap.add_argument("--min-gap", type=float, default=20.0,
                    help="seconds between visits to count as a revisit")
    ap.add_argument("--tol", type=float, default=1.0,
                    help="horizontal radius [m] counting as the same place")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    runs = []
    for f in args.files:
        if not os.path.exists(f):
            print("missing: %s" % f)
            continue
        runs.append(load(f))
    if not runs:
        sys.exit("nothing to check")
    # The baseline is the unfiltered run: everything else is measured against
    # it
    runs.sort(key=lambda r: (0 if "L0" in r[0] else 1, r[0]))
    base = runs[0]
    if args.vert:
        vert = AXES.index(args.vert)
        _, sp = pick_vertical(base[2])
        print("\nvertical axis forced to '%s'" % args.vert)
    else:
        vert, sp = pick_vertical(base[2])
        print("\nvertical axis detected as '%s' from %s (spreads "
              "x %.2f  y %.2f  z %.2f m)"
              % (AXES[vert], base[0], sp[0], sp[1], sp[2]))
        print("  For /aft_mapped_to_init, 'y' is correct -- LeGO-LOAM publishes")
        print("  LOAM's internal frame, in which y is up.")
    plane = [i for i in range(3) if i != vert]

    hdr = ("%-12s %7s %9s %9s %10s %9s %7s %10s"
           % ("run", "poses", "vert[m]", "net[m]", "revisit", "mismatch",
              "pairs", "start-end"))
    print("")
    print(hdr)
    print("-" * len(hdr))

    rows = []
    for name, t, p in runs:
        xy, z = p[:, plane], p[:, vert]
        v_range = float(np.ptp(z))
        v_net = float(z[-1] - z[0])
        sep, i, j = closest_revisit(xy, t, args.min_gap)
        dz, dt = overlap_mismatch(xy, z, t, args.min_gap, args.tol)
        mism = float(np.median(dz)) if dz.size else float("nan")
        s2e = float(np.linalg.norm(xy[-1] - xy[0]))
        rows.append((name, t, xy, z, v_range, v_net, sep, i, j, dz, dt,
                     mism, s2e))
        print("%-12s %7d %9.2f %9.2f %10s %9s %7d %10.2f"
              % (name, len(t), v_range, v_net,
                 "%.2f m" % sep if np.isfinite(sep) else "none",
                 "%.2f m" % mism if mism == mism else "-",
                 dz.size, s2e))

    print("""
vert       total height range over the run
net        height at the end minus height at the start
revisit    closest the path comes to a place it visited >%.0f s earlier
mismatch   median height disagreement over ALL pose pairs that pass within
           %.1f m of each other more than %.0f s apart
pairs      how many such pairs there were -- a handful means the path barely
           overlaps itself and this test is weak for that run
start-end  horizontal distance from first pose to last""" % (
        args.min_gap, args.tol, args.min_gap))

    # The verdict, for the baseline only -- it is the one every other row is
    # measured against.
    name, t, xy, z, v_range, v_net, sep, i, j, dz, dt, mism, s2e = rows[0]
    print("\n" + "=" * len(hdr))
    print("VERDICT for the baseline (%s)" % name)
    print("=" * len(hdr))
    if not dz.size:
        print("""The path never revisits a place, so this test cannot run on it.
Without a revisit there is no way to separate terrain from vertical drift using
the trajectory alone""")
    elif mism != mism:
        print("no usable pairs")
    else:
        frac = mism / max(v_range, 1e-6)
        print("%d overlapping pose pairs, median height mismatch %.3f m "
              "against a total height range of %.2f m (%.0f%%)."
              % (dz.size, mism, v_range, 100 * frac))
        if mism < 0.10:
            print("""
TERRAIN. The run reports the same height each time it passes the same place, so
the height profile is the crater and the mounds, not drift. The baseline is
sound and the drift numbers measured against it stand.""")
        elif mism < 0.30:
            print("""
MOSTLY TERRAIN, with some vertical drift.""")
        else:
            print("""
VERTICAL DRIFT. The baseline reports materially different heights for the same
ground, so L0 is itself drifting and every drift number measured against it is a
looser lower bound than it appears.""")
        if dz.size > 8:
            # Drift grows with elapsed time between the visits; terrain error
            # does not. The correlation separates them independently of the
            # magnitude threshold above.
            r = float(np.corrcoef(dt, dz)[0, 1]) if np.ptp(dt) > 1e-6 else 0.0
            print("\nmismatch vs elapsed time between visits: r = %+.2f" % r)
            print("   r near 0    the error is a property of the PLACE "
                  "-> terrain")
            print("   r positive  the error grows with elapsed time "
                  "-> drift")

    print("""
Start-to-end is the absolute check that needs no baseline: the operator tried to
finish each loop where it began.""")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        fig, axes = plt.subplots(1, 3, figsize=(15.5, 4.6))

        ax = axes[0]
        for k, r in enumerate(rows):
            ax.plot(r[1], r[3], lw=1.3, color=SERIES[k % len(SERIES)],
                    label=r[0])
        ax.set_xlabel("time [s]")
        ax.set_ylabel("height, %s axis [m]" % AXES[vert])
        ax.set_title("Height vs time\n(a ramp is drift; bumps are terrain)",
                     fontsize=10)
        ax.grid(True, color=GRID, lw=0.7)
        ax.set_axisbelow(True)
        ax.legend(fontsize=8, frameon=False)

        # Height as a function of PLACE for the baseline. If the profile is
        # terrain, the two passes over the same ground lie on top of each other.
        ax = axes[1]
        name, t, xy, z = rows[0][0], rows[0][1], rows[0][2], rows[0][3]
        sc = ax.scatter(xy[:, 0], xy[:, 1], c=z, s=14, cmap="viridis")
        ax.plot(xy[0, 0], xy[0, 1], "o", ms=9, mfc="none", mec=INK, mew=1.6)
        ax.plot(xy[-1, 0], xy[-1, 1], "x", ms=10, color=INK, mew=1.8)
        ax.set_aspect("equal")
        ax.set_xlabel("%s [m]" % AXES[plane[0]])
        ax.set_ylabel("%s [m]" % AXES[plane[1]])
        ax.set_title("%s height by place\n(same colour at a crossing = "
                     "terrain)" % name, fontsize=10)
        ax.grid(True, color=GRID, lw=0.7)
        ax.set_axisbelow(True)
        fig.colorbar(sc, ax=ax, label="height [m]")

        ax = axes[2]
        dz, dt = rows[0][9], rows[0][10]
        if dz.size:
            ax.plot(dt, dz, "o", ms=4, color=SERIES[0], alpha=0.6)
            ax.axhline(0.10, color=INK_MUTED, lw=1.0, ls="--")
            ax.text(ax.get_xlim()[0], 0.105, " 0.10 m: terrain below, "
                    "drift above", fontsize=8, color=INK_MUTED, va="bottom")
            ax.set_xlabel("elapsed time between the two visits [s]")
            ax.set_ylabel("height mismatch [m]")
            ax.set_title("%s: does the mismatch grow with time?\n"
                         "(flat = terrain, rising = drift)" % name,
                         fontsize=10)
        else:
            ax.text(0.5, 0.5, "no overlapping passes", ha="center",
                    va="center", transform=ax.transAxes, color=INK_MUTED)
            ax.set_title("no revisit to test", fontsize=10)
        ax.grid(True, color=GRID, lw=0.7)
        ax.set_axisbelow(True)

        fig.tight_layout()
        fig.savefig(args.plot, dpi=130, bbox_inches="tight")
        print("\nwrote %s" % args.plot)


if __name__ == "__main__":
    main()
