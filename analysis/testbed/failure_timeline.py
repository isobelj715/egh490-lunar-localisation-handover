#!/usr/bin/env python3
"""
What is the scene doing at the moment the odometry starts going wrong?

Spearman (rank) correlation is used, so a monotonic but non-linear relationship
still shows up.

Usage:
    python3 failure_timeline.py \
        --baseline results/testbed/ellipse1/est_raw_L0.txt \
        --est results/testbed/ellipse1/est_raw_L2.txt \
        --structure results/testbed/ellipse1/logs/L2/structure.csv \
        --plot ellipse1_L2_timeline.png

    # compare the same level with the suppression removed
    python3 failure_timeline.py \
        --baseline results/testbed/ellipse1/est_raw_L0.txt \
        --est results/testbed/ellipse1_m0f10/est_raw_L2.txt \
        --structure results/testbed/ellipse1_m0f10/logs/L2/structure.csv \
        --plot ellipse1_L2_nobrake_timeline.png
"""

import argparse
import os
import sys

import numpy as np

GRID, INK, INK_MUTED = "#d8d8d4", "#1a1a1a", "#6b6b6b"
SERIES = ["#1f77b4", "#ff7f0e", "#2ca02c", "#d62728", "#9467bd"]

TIME_NAMES = ("t", "time", "stamp", "timestamp", "sec", "secs", "seconds")
# Columns worth lining up against the error, in the order they are plotted.
WANTED = [
    ("n_corr", "correspondences", "how much usable structure is in view"),
    ("map_full_lambda_min", "lambda_min", "how well the pose is constrained"),
    ("residual_rms_init", "residual rms", "fit quality before optimising"),
    ("iter_count", "LM iterations", "how hard the solver worked"),
]


def load_poses(path):
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 4:
        sys.exit("%s: expected at least 4 columns (t x y z ...)" % path)
    return a[:, 0], a[:, 1:4]


def ground_plane(p):
    return [i for i in range(3) if i != int(np.argmin(np.ptp(p, axis=0)))]


def load_rotations(path):
    """Rotation matrices from a TUM file, or None if it carries no quaternion.

    R[:, :, k] is where body axis k points in world coordinates.
    """
    a = np.loadtxt(path, ndmin=2)
    if a.shape[1] < 8:
        return None
    q = a[:, 4:8]
    n = np.linalg.norm(q, axis=1, keepdims=True)
    if not np.all(n > 1e-9):
        return None
    q = q / n
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
    return R


def turn_rate(path, rt, plane, vert, axis, half=0.5):
    R = load_rotations(path)
    if R is None:
        print("   baseline carries no quaternion -- no turn rate panel")
        return None
    k = "xyz".index(axis)
    v = R[:, :, k]
    flat = np.hypot(v[:, plane[0]], v[:, plane[1]])
    tilt = float(np.degrees(np.median(np.arctan2(np.abs(v[:, vert]), flat))))
    if tilt > 60:
        others = " or ".join("--axis %s" % c for c in "xyz" if c != axis)
        print("   body axis '%s' lies %.0f deg out of the ground plane, so "
              "its heading is\n   meaningless -- no turn rate panel. Re-run "
              "with %s." % (axis, tilt, others))
        return None
    hd = np.unwrap(np.arctan2(v[:, plane[1]], v[:, plane[0]]))
    rate = np.degrees(np.abs(np.interp(rt + half, rt, hd)
                             - np.interp(rt - half, rt, hd))) / (2.0 * half)
    print("   turn rate off body axis '%s' (%.0f deg out of plane): median "
          "%.1f deg/s, peak %.1f" % (axis, tilt, np.median(rate), rate.max()))
    return rate


def read_structure(path, t0, t1):
    """structure.csv as a dict of column -> array, plus a time array in
    seconds-since-start aligned to the pose clock."""
    with open(path) as fh:
        head = fh.readline().strip().split(",")
    data = np.genfromtxt(path, delimiter=",", skip_header=1, dtype=float)
    if data.ndim == 1:
        data = data[None, :]
    cols = {h.strip(): data[:, i] for i, h in enumerate(head)
            if i < data.shape[1]}

    tcol = next((h for h in head if h.strip().lower() in TIME_NAMES), None)
    if tcol is not None:
        t = cols[tcol].astype(float)
        # The log may carry absolute ROS time or seconds-since-start; normalise
        # by comparing its span to the trajectory's.
        if t[0] > 1e8:
            t = t - t[0]
        print("   structure.csv: %d rows, time column '%s', span %.1f s"
              % (len(t), tcol.strip(), t[-1] - t[0]))
    else:
        # When timestamps are unavailable, assume uniformly spaced
        # mapping iterations. Dropped iterations may cause time
        # alignment errors.
        t = np.linspace(0.0, t1 - t0, len(data))
        print("   structure.csv: %d rows, NO time column -- rows assumed "
              "evenly spaced" % len(data))
        print("   !! that is an assumption; a skipped mapping iteration would "
              "shift the alignment")
    return cols, t, (tcol is not None)


def local_rpe(t, est, ref, window_m):
    """Error produced within a sliding window of `window_m` metres of travel.

    Measured along the REFERENCE path so every run is evaluated over the same
    physical stretches. Reported at the midpoint of each window.
    """
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(ref, axis=0), axis=1))]
    mid, err = [], []
    j = 0
    for i in range(len(s)):
        while j < len(s) and s[j] - s[i] < window_m:
            j += 1
        if j >= len(s):
            break
        de = est[j] - est[i]
        dr = ref[j] - ref[i]
        mid.append(0.5 * (t[i] + t[j]))
        err.append(float(np.linalg.norm(de - dr)))
    return np.array(mid), np.array(err)


def spearman(a, b):
    ok = np.isfinite(a) & np.isfinite(b)
    if ok.sum() < 8:
        return float("nan")
    ra = np.argsort(np.argsort(a[ok])).astype(float)
    rb = np.argsort(np.argsort(b[ok])).astype(float)
    ra -= ra.mean()
    rb -= rb.mean()
    d = np.sqrt((ra * ra).sum() * (rb * rb).sum())
    return float((ra * rb).sum() / d) if d > 0 else float("nan")


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--baseline", required=True)
    ap.add_argument("--est", required=True)
    ap.add_argument("--structure", default=None)
    ap.add_argument("--window", type=float, default=2.0,
                    help="metres of travel per local-error window")
    ap.add_argument("--fail-at", type=float, default=0.5,
                    help="divergence [m] counted as tracking lost")
    ap.add_argument("--still-speed", type=float, default=0.05,
                    help="m/s below which a window counts as parked and is "
                         "dropped from the 'while moving' correlation")
    ap.add_argument("--axis", default="z", choices=["x", "y", "z"],
                    help="body axis taken as forward, for the turn rate; 'z' "
                         "for LeGO-LOAM, and the tilt check will tell you if "
                         "it is wrong")
    ap.add_argument("--plot", default=None)
    args = ap.parse_args()

    for f in (args.baseline, args.est):
        if not os.path.exists(f):
            sys.exit("missing: %s" % f)

    rt, rp = load_poses(args.baseline)
    plane = ground_plane(rp)
    rxy = rp[:, plane]
    et, ep = load_poses(args.est)
    exy = ep[:, plane]

    t0 = rt[0]
    t = et - t0
    ref_at = np.column_stack([np.interp(et, rt, rxy[:, k]) for k in (0, 1)])
    cum = np.linalg.norm(exy - ref_at, axis=1)

    # Rover speed from the BASELINE, so it is the real speed rather than the
    # estimate's idea of it.
    rs = np.r_[0.0, np.linalg.norm(np.diff(rxy, axis=0), axis=1)]
    rdt = np.r_[1e-6, np.diff(rt)]
    rspeed = rs / np.maximum(rdt, 1e-6)
    speed = np.interp(et, rt, rspeed)

    # Turn rate, also from the BASELINE, for the same reason.
    vert = [i for i in range(3) if i not in plane][0]
    turn = turn_rate(args.baseline, rt, plane, vert, args.axis)

    lt, lerr = local_rpe(et, exy, ref_at, args.window)
    lt = lt - t0

    print("\n%s" % os.path.basename(args.est))
    print("   %d poses over %.1f s; local error window %.1f m"
          % (len(et), t[-1] - t[0], args.window))

    crossed = np.flatnonzero(cum > args.fail_at)
    t_fail = float(t[crossed[0]]) if len(crossed) else None
    if t_fail is not None:
        print("   divergence first exceeds %.2f m at t = %.1f s"
              % (args.fail_at, t_fail))
    else:
        print("   divergence never exceeds %.2f m -- this run tracked"
              % args.fail_at)

    cols, st, had_time = ({}, None, False)
    if args.structure and os.path.exists(args.structure):
        cols, st, had_time = read_structure(args.structure, t0, et[-1])
    elif args.structure:
        print("   structure log not found: %s" % args.structure)

    # Line the scene metrics up with the local error windows.
    series = []
    for key, label, why in WANTED:
        if key in cols:
            v = np.interp(lt, st, cols[key])
            series.append((key, label, why, v))
    sp = np.interp(lt, t, speed)
    series.append(("speed", "rover speed", "true speed from the baseline", sp))
    if turn is not None:
        series.append(("turn", "turn rate",
                       "|heading rate| from the baseline",
                       np.interp(lt, t, np.interp(et, rt, turn))))

    if series:
        pre = (lt < t_fail) if t_fail is not None else np.ones(len(lt), bool)
        moving = sp > args.still_speed
        n_pre, n_mov = int(pre.sum()), int(moving.sum())
        r_et = spearman(lerr[moving], lt[moving]) if n_mov >= 8 else float("nan")
        print("\n   Spearman correlation of LOCAL error against:")
        print("   %-22s %10s %11s %12s %9s %12s"
              % ("", "whole run", "before fail", "while moving", "vs time",
                 "time removed"))
        print("   %-22s %10s %11s %12s %9s %12s"
              % ("", "n=%d" % len(lt), "n=%d" % n_pre, "n=%d" % n_mov, "", ""))
        print("   " + "-" * 81)

        def cell(mask, v, n):
            if n < 8:
                return "too few"
            r = spearman(v[mask], lerr[mask])
            return "%+.2f" % r if r == r else "n/a"

        def saturated(v):
            """A channel pinned at a cap carries almost no rank information."""
            u = np.unique(v)
            if len(u) < 4:
                return True
            top = float((v == u[-1]).mean())
            return top > 0.6

        flagged = []
        for key, label, why, v in series:
            if n_mov >= 8:
                r_vt = spearman(v[moving], lt[moving])
                r_ve = spearman(v[moving], lerr[moving])
                d = (1 - r_vt ** 2) * (1 - r_et ** 2)
                r_p = ((r_ve - r_vt * r_et) / np.sqrt(d)) if d > 1e-9 \
                    else float("nan")
                s_vt = "%+.2f" % r_vt if r_vt == r_vt else "n/a"
                if r_vt == r_vt and abs(r_vt) > 0.995:
                    # The channel IS the clock. Nothing of it is independent
                    # of time, so there is no partial to compute -- and that
                    # is the finding, not a failure.
                    s_p = "= time"
                else:
                    s_p = "%+.2f" % r_p if r_p == r_p else "n/a"
            else:
                s_vt, s_p = "too few", "too few"
            mark = ""
            if n_mov >= 8 and saturated(v[moving]):
                mark, _ = " !", flagged.append(label)
            print("   %-22s %+10.2f %11s %12s %9s %12s%s  %s"
                  % (label, spearman(v, lerr), cell(pre, v, n_pre),
                     cell(moving, v, n_mov), s_vt, s_p, mark, why))

        if r_et == r_et:
            print("\n   the LOCAL ERROR itself is %+.2f against time while "
                  "moving." % r_et)
            if abs(r_et) > 0.995:
                print("   It is perfectly monotone in time, so holding time "
                      "fixed leaves nothing to\n   correlate and the last "
                      "column cannot be computed. That is itself the "
                      "answer:\n   on this run the error is a trend, and no "
                      "channel can explain more than the trend.")
            elif abs(r_et) > 0.5:
                print("   That trend is strong, so every channel that also "
                      "trends will correlate with\n   it for free. Read the "
                      "'time removed' column and ignore 'while moving'.")
        if flagged:
            print("\n   ! %s sits at one value for most of the run, so its "
                  "rank correlation is\n     driven by a handful of points. "
                  "Do not quote it."
                  % " and ".join(flagged))

        if t_fail is not None and n_pre and not moving[pre].any():
            print("\n   !! every 'before fail' window is a parked one, so that "
                  "column is a\n      correlation against a stationary rover. "
                  "Ignore it and read 'while moving'.")
        if n_mov and n_mov < 12:
            print("\n   !! only %d moving windows -- 'while moving' is thin. "
                  "Treat anything\n      under about 0.6 as nothing." % n_mov)

    if "is_degenerate" in cols:
        dg = np.interp(lt, st, cols["is_degenerate"])
        frac = float((dg > 0.5).mean())
        print("\n   suppression active for %.0f%% of the windows" % (100 * frac))
        if 0.02 < frac < 0.98:
            on, off = lerr[dg > 0.5], lerr[dg <= 0.5]
            if len(on) >= 5 and len(off) >= 5:
                print("   median local error while suppressed   %.3f m (n=%d)"
                      % (float(np.median(on)), len(on)))
                print("   median local error while not          %.3f m (n=%d)"
                      % (float(np.median(off)), len(off)))
                print("""   This run has both regimes in it, so that is a within-run comparison of
   the suppression's effect -- no second build required.""")

    if args.plot:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt

        panels = 2 + len(series)
        fig, axes = plt.subplots(panels, 1, figsize=(11, 2.0 * panels),
                                 sharex=True)

        ax = axes[0]
        ax.plot(t, cum, lw=1.3, color=SERIES[3])
        ax.set_ylabel("cumulative\nerror [m]")
        ax.set_yscale("symlog", linthresh=0.05)

        ax = axes[1]
        ax.plot(lt, lerr, lw=1.2, color=SERIES[0])
        ax.set_ylabel("local error\n[m per %.0f m]" % args.window)

        for k, (key, label, why, v) in enumerate(series):
            ax = axes[2 + k]
            ax.plot(lt, v, lw=1.2, color=SERIES[(k + 1) % len(SERIES)])
            ax.set_ylabel(label.replace(" ", "\n"))
            if key == "map_full_lambda_min":
                ax.set_yscale("log")

        for ax in axes:
            ax.grid(True, color=GRID, lw=0.6)
            ax.set_axisbelow(True)
            if t_fail is not None:
                ax.axvline(t_fail, color=INK, lw=1.1, ls="--")
        if t_fail is not None:
            axes[0].text(t_fail, axes[0].get_ylim()[1], " tracking lost",
                         fontsize=8, color=INK, va="top")
        else:
            axes[0].text(0.99, 0.92, "tracked throughout", fontsize=8,
                         color=INK_MUTED, ha="right",
                         transform=axes[0].transAxes)

        # The rover is not always moving. A stationary stretch makes a
        # fixed-distance error window span a long time, which confounds any
        # correlation with speed -- and is itself worth seeing, because error
        # accumulated while parked cannot be a tracking problem.
        still = np.interp(lt, t, speed) < 0.05
        if still.any():
            for ax in axes:
                ax.fill_between(lt, 0, 1, where=still, transform=
                                ax.get_xaxis_transform(), color="#f0c674",
                                alpha=0.25, lw=0)
            axes[1].text(0.01, 0.92, "shaded: rover stationary", fontsize=8,
                         color=INK_MUTED, transform=axes[1].transAxes)
        axes[-1].set_xlabel("t [s]")
        fig.suptitle("%s — local error against what the sensor was seeing"
                     % os.path.basename(args.est), fontsize=11)
        fig.tight_layout()
        fig.savefig(args.plot, dpi=130, bbox_inches="tight")
        print("\nwrote %s" % args.plot)


if __name__ == "__main__":
    main()
