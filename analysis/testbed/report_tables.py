#!/usr/bin/env python3
"""
report_tables.py -- the tables the report needs that no plotting script emits.

WHAT IT PRODUCES
  1. per-run summary      session, shape, path length, duration, parked
                          window, mean speed
  2. lambda_min           median per run per level, against eignThre
  3. threshold firing     fraction of MAPPED scans where is_degenerate fired,
                          per run per level
  4. path length          est_path_len_m per run per level, and the inflation
                          against that run's L0
Usage:
    python3 report_tables.py --root results/testbed \
        --run september:september_lim --run rectangle:rectangle_lim \
        --run ellipse1:ellipse1_lim --run zigzag:zigzag_lim \
        --csv-dir report/tables
"""

import argparse
import csv
import json
import os
import sys

import numpy as np

# Facts about the drives that are not recorded anywhere in results
RUNS = {
    "september": {"session": "2026-09-16", "shape": "open traverse"},
    "rectangle": {"session": "2026-10-07", "shape": "closed rectangle"},
    "ellipse1":  {"session": "2026-10-07", "shape": "closed ellipse"},
    "zigzag":    {"session": "2026-10-07", "shape": "open zigzag"},
}

LEVELS = ["L0", "L1", "L2", "L3"]
NEEDED = ("is_degenerate", "map_ran", "map_full_lambda_min")


def read_meta(path):
    try:
        with open(path) as fh:
            return json.load(fh)
    except (OSError, ValueError):
        return None


def read_structure(path):
    """-> (n_rows, n_mapped, fired_fraction, median lambda_min) or None.

    structure.csv carries over a hundred columns, so pull the three that
    matter by header index and stream the rest past.
    """
    try:
        fh = open(path, newline="")
    except OSError:
        return None
    with fh:
        rd = csv.reader(fh)
        try:
            head = next(rd)
        except StopIteration:
            return None
        missing = [c for c in NEEDED if c not in head]
        if missing:
            print("   ! %s lacks %s" % (path, ", ".join(missing)),
                  file=sys.stderr)
            return None
        i_deg, i_ran, i_lam = (head.index(c) for c in NEEDED)
        n, mapped, fired, lam = 0, 0, 0, []
        for row in rd:
            if len(row) <= max(i_deg, i_ran, i_lam):
                continue
            n += 1
            if (row[i_ran] or "0").strip() not in ("1", "1.0", "True", "true"):
                continue
            mapped += 1
            if (row[i_deg] or "").strip() in ("1", "1.0", "True", "true"):
                fired += 1
            v = (row[i_lam] or "").strip()
            if v:
                try:
                    f = float(v)
                except ValueError:
                    continue
                if np.isfinite(f):
                    lam.append(f)
    if not mapped:
        return n, 0, float("nan"), float("nan")
    return (n, mapped, fired / mapped,
            float(np.median(lam)) if lam else float("nan"))


def load_tum(path):
    try:
        a = np.loadtxt(path, ndmin=2)
    except (OSError, ValueError):
        return None
    if a.shape[1] < 4 or len(a) < 3:
        return None
    return a[:, 0], a[:, 1:4]


def motion(path, still):
    """Parked-window length and mean speed from the unfiltered pass.
    """
    got = load_tum(path)
    if got is None:
        return {}
    t, p = got
    plane = [i for i in range(3) if i != int(np.argmin(np.ptp(p, axis=0)))]
    step = np.linalg.norm(np.diff(p[:, plane], axis=0), axis=1)
    dt = np.maximum(np.diff(t), 1e-6)
    v = step / dt
    tm = 0.5 * (t[1:] + t[:-1])
    moving = v > still

    def leading(mask):
        idx = np.flatnonzero(mask)
        return float(tm[idx[0]] - tm[0]) if len(idx) else float(tm[-1] - tm[0])

    def trailing(mask):
        idx = np.flatnonzero(mask)
        return float(tm[-1] - tm[idx[-1]]) if len(idx) else 0.0

    return {
        "park_open_s": leading(moving),
        "park_close_s": trailing(moving),
        "mean_speed": float(v[moving].mean()) if moving.any() else float("nan"),
        "peak_speed": float(v.max()),
    }


def fmt(x, nd=2, dash="--"):
    if x is None or (isinstance(x, float) and not np.isfinite(x)):
        return dash
    return ("%%.%df" % nd) % x


def table(rows, head, out=sys.stdout):
    w = [max(len(str(h)), *(len(str(r[i])) for r in rows)) if rows
         else len(str(h)) for i, h in enumerate(head)]
    print("| " + " | ".join(str(h).ljust(w[i]) for i, h in enumerate(head))
          + " |", file=out)
    print("|" + "|".join("-" * (x + 2) for x in w) + "|", file=out)
    for r in rows:
        print("| " + " | ".join(str(c).ljust(w[i]) for i, c in enumerate(r))
              + " |", file=out)


def write_csv(path, head, rows):
    with open(path, "w", newline="") as fh:
        w = csv.writer(fh)
        w.writerow(head)
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--root", default="results/testbed")
    ap.add_argument("--run", action="append", default=[], metavar="NAME:DIR",
                    help="display name and its stock directory; repeatable. "
                         "A run whose levels live in two directories takes "
                         "two entries, e.g. september:september_lim and "
                         "september-L2:september_v2")
    ap.add_argument("--levels", nargs="+", default=LEVELS)
    ap.add_argument("--still", type=float, default=0.05,
                    help="m/s below which the rover counts as parked")
    ap.add_argument("--threshold", type=float, default=100.0,
                    help="eignThre, for the over/under column")
    ap.add_argument("--csv-dir", default=None)
    a = ap.parse_args()

    if not a.run:
        sys.exit("give at least one --run NAME:DIR")
    pairs = []
    for spec in a.run:
        if ":" not in spec:
            sys.exit("--run wants NAME:DIR, got %r" % spec)
        name, d = spec.split(":", 1)
        pairs.append((name, os.path.join(a.root, d)))

    for name, d in pairs:
        if not os.path.isdir(d):
            print("   ! no such directory: %s" % d, file=sys.stderr)

    if a.csv_dir:
        os.makedirs(a.csv_dir, exist_ok=True)

    # ---- 1. per-run summary -------------------------------------------
    head1 = ["run", "session", "shape", "path L0 [m]", "duration [s]",
             "parked open [s]", "parked close [s]", "mean speed [m/s]",
             "poses L0"]
    rows1 = []
    for name, d in pairs:
        meta = read_meta(os.path.join(d, "meta_L0.json")) or {}
        mo = motion(os.path.join(d, "est_raw_L0.txt"), a.still)
        info = RUNS.get(name.split("-")[0], {})
        rows1.append([
            name, info.get("session", "--"), info.get("shape", "--"),
            fmt(meta.get("est_path_len_m")), fmt(meta.get("est_duration_s"), 1),
            fmt(mo.get("park_open_s"), 1), fmt(mo.get("park_close_s"), 1),
            fmt(mo.get("mean_speed")), meta.get("est_poses", "--"),
        ])

    # ---- 2 & 3. lambda_min and firing fraction ------------------------
    head2 = ["run"] + ["%s median lambda_min" % L for L in a.levels]
    head3 = ["run"] + ["%s fires [%%]" % L for L in a.levels]
    head3b = ["run"] + ["%s mapped scans" % L for L in a.levels]
    rows2, rows3, rows3b = [], [], []
    for name, d in pairs:
        r2, r3, r3b = [name], [name], [name]
        for L in a.levels:
            got = read_structure(os.path.join(d, "logs", L, "structure.csv"))
            if got is None:
                r2.append("--")
                r3.append("--")
                r3b.append("--")
                continue
            _, mapped, frac, lam = got
            over = ""
            if np.isfinite(lam):
                over = " (over)" if lam > a.threshold else ""
            r2.append(fmt(lam, 1) + over)
            r3.append(fmt(100.0 * frac, 1) if np.isfinite(frac) else "--")
            r3b.append(str(mapped))
        rows2.append(r2)
        rows3.append(r3)
        rows3b.append(r3b)

    # ---- 4. path length per level, and inflation ----------------------
    head4 = ["run"] + ["%s [m]" % L for L in a.levels] + ["L3 / L0"]
    rows4 = []
    for name, d in pairs:
        r, base = [name], None
        for L in a.levels:
            meta = read_meta(os.path.join(d, "meta_%s.json" % L))
            v = meta.get("est_path_len_m") if meta else None
            if L == "L0":
                base = v
            r.append(fmt(v))
        last = read_meta(os.path.join(d, "meta_L3.json"))
        lv = last.get("est_path_len_m") if last else None
        r.append(fmt(lv / base, 2) + "x" if (lv and base) else "--")
        rows4.append(r)

    print("\n### 1. Per-run summary  (from meta_L0.json and est_raw_L0.txt)\n")
    table(rows1, head1)
    print("\n### 2. Median lambda_min per level  (mapped scans only; "
          "eignThre = %g)\n" % a.threshold)
    table(rows2, head2)
    print("\n### 3. Scans on which the threshold fires  (% of MAPPED "
          "scans)\n")
    table(rows3, head3)
    print("\n   base for those percentages:\n")
    table(rows3b, head3b)
    print("\n### 4. Path length per level, and L3 against L0\n")
    table(rows4, head4)
    print("""
   Path length is est_path_len_m, the length of the ESTIMATED trajectory. """)

    if a.csv_dir:
        write_csv(os.path.join(a.csv_dir, "summary.csv"), head1, rows1)
        write_csv(os.path.join(a.csv_dir, "lambda_min.csv"), head2, rows2)
        write_csv(os.path.join(a.csv_dir, "threshold_fires.csv"), head3, rows3)
        write_csv(os.path.join(a.csv_dir, "mapped_scans.csv"), head3b, rows3b)
        write_csv(os.path.join(a.csv_dir, "path_length.csv"), head4, rows4)
        print("\n   wrote 5 CSVs to %s" % a.csv_dir)


if __name__ == "__main__":
    main()
