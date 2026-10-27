#!/usr/bin/env python3
"""
derive_structure.py

Merges the per-scan logs written by the instrumented LeGO-LOAM nodes and
derives every candidate structure metric from the raw quantities.

Inputs (produced by scan_logger.h, one directory per run):
    <run_dir>/features.csv      -- from featureAssociation
    <run_dir>/conditioning.csv  -- from mapOptmization

Output:
    <run_dir>/structure.csv     -- one row per mapping update, with derived metrics

Usage:
    python3 derive_structure.py <run_dir> [--tol 0.25] [--frame loam|sensor]
    python3 derive_structure.py --batch <parent_dir_of_many_runs>
"""

import argparse
import os
import sys

import numpy as np
import pandas as pd


# ---------------------------------------------------------------------------
# Loading
# ---------------------------------------------------------------------------

def load_run(run_dir):
    fp = os.path.join(run_dir, "features.csv")
    cp = os.path.join(run_dir, "conditioning.csv")
    feat = pd.read_csv(fp) if os.path.exists(fp) else None
    cond = pd.read_csv(cp) if os.path.exists(cp) else None
    if feat is None and cond is None:
        raise FileNotFoundError(
            "neither features.csv nor conditioning.csv found in %s" % run_dir)
    for name, df in (("features.csv", feat), ("conditioning.csv", cond)):
        if df is not None and "t" not in df.columns:
            raise ValueError("%s in %s has no 't' column" % (name, run_dir))
    return feat, cond


def merge_on_time(cond, feat, tol):
    """Nearest-timestamp join. mapOptmization runs at a lower rate than
    featureAssociation (mappingProcessInterval gates it), so conditioning rows
    are sparser."""
    if cond is None:
        return feat.copy()
    if feat is None:
        return cond.copy()
    left = cond.sort_values("t").reset_index(drop=True)
    right = feat.sort_values("t").reset_index(drop=True)
    right = right.rename(columns={"t": "t_feat"})
    merged = pd.merge_asof(
        left, right,
        left_on="t", right_on="t_feat",
        direction="nearest", tolerance=float(tol),
    )
    merged["join_dt"] = merged["t"] - merged["t_feat"]
    n_missed = int(merged["t_feat"].isna().sum())
    if n_missed:
        sys.stderr.write(
            "  note: %d/%d conditioning rows had no feature row within %.3f s\n"
            % (n_missed, len(merged), tol))
    return merged


def columns_with_prefix(df, prefix):
    cols = [c for c in df.columns if c.startswith(prefix + "_")]
    return sorted(cols)


# ---------------------------------------------------------------------------
# Histogram-derived metrics
# ---------------------------------------------------------------------------

def hist_metrics(df, prefix, out, circular=True):
    """Occupancy, normalised Shannon entropy, and (for circular histograms)
    the largest contiguous empty arc. The empty-arc metric is the direct
    quantitative expression of 'all my structure is on one side'."""
    cols = columns_with_prefix(df, prefix)
    if not cols:
        return
    H = df[cols].to_numpy(dtype=float)
    n_bins = H.shape[1]
    total = H.sum(axis=1)
    with np.errstate(divide="ignore", invalid="ignore"):
        P = np.where(total[:, None] > 0, H / total[:, None], 0.0)
        logP = np.where(P > 0, np.log(P), 0.0)
        entropy = -(P * logP).sum(axis=1) / np.log(n_bins)
    occupied = (H > 0).sum(axis=1)

    out[prefix + "_total"] = total
    out[prefix + "_occupied_bins"] = occupied
    out[prefix + "_occupancy_frac"] = occupied / float(n_bins)
    out[prefix + "_entropy_norm"] = np.where(total > 0, entropy, np.nan)

    if circular:
        gaps = np.zeros(H.shape[0])
        for i in range(H.shape[0]):
            occ = H[i] > 0
            if not occ.any():
                gaps[i] = 1.0
                continue
            if occ.all():
                gaps[i] = 0.0
                continue
            doubled = np.concatenate([occ, occ])
            best = run = 0
            for v in doubled:
                run = 0 if v else run + 1
                best = max(best, run)
            gaps[i] = min(best, n_bins) / float(n_bins)
        out[prefix + "_max_empty_arc_frac"] = gaps


# ---------------------------------------------------------------------------
# Matrix-derived metrics
# ---------------------------------------------------------------------------

def read_sym_matrix(df, prefix, n):
    """Rebuild an n x n symmetric matrix per row from the upper-triangle columns
    written by setSymMatrix. Returns array of shape (rows, n, n), or None."""
    M = np.full((len(df), n, n), np.nan)
    found = False
    for r in range(n):
        for c in range(r, n):
            col = "%s_r%dc%d" % (prefix, r, c)
            if col not in df.columns:
                continue
            found = True
            v = df[col].to_numpy(dtype=float)
            M[:, r, c] = v
            M[:, c, r] = v
    return M if found else None


def read_full_matrix(df, prefix, n):
    M = np.full((len(df), n, n), np.nan)
    found = False
    for r in range(n):
        for c in range(n):
            col = "%s_r%dc%d" % (prefix, r, c)
            if col not in df.columns:
                continue
            found = True
            M[:, r, c] = df[col].to_numpy(dtype=float)
    return M if found else None


def conditioning_metrics(M, out, tag, rot_idx=None, trans_idx=None):
    """Derive conditioning metrics from a stack of approximate Hessians.
    """
    rows = M.shape[0]

    def block_metrics(sub, name):
        lmin = np.full(rows, np.nan)
        lmax = np.full(rows, np.nan)
        trace = np.full(rows, np.nan)
        logdet = np.full(rows, np.nan)
        for i in range(rows):
            A = sub[i]
            if not np.all(np.isfinite(A)):
                continue
            try:
                w = np.linalg.eigvalsh((A + A.T) / 2.0)
            except np.linalg.LinAlgError:
                continue
            w = np.sort(w)
            lmin[i] = w[0]
            lmax[i] = w[-1]
            trace[i] = w.sum()
            wp = w[w > 0]
            logdet[i] = np.sum(np.log(wp)) if len(wp) == len(w) else np.nan
        with np.errstate(divide="ignore", invalid="ignore"):
            cond = np.where(lmin > 0, lmax / lmin, np.inf)
            # Two distinct scale-free quantities, kept separate and named for
            # what they actually are:
            #   ratio_lmax  = lambda_min / lambda_max  (inverse condition number)
            #   ratio_trace = lambda_min / sum(lambda) (share of total information
            #                 held by the weakest direction)
            ratio_lmax = np.where(lmax > 0, lmin / lmax, np.nan)
            ratio_trace = np.where(trace > 0, lmin / trace, np.nan)
        out["%s_%s_lambda_min" % (tag, name)] = lmin
        out["%s_%s_lambda_max" % (tag, name)] = lmax
        out["%s_%s_trace" % (tag, name)] = trace
        out["%s_%s_cond" % (tag, name)] = cond
        out["%s_%s_logdet" % (tag, name)] = logdet
        out["%s_%s_ratio_lmax" % (tag, name)] = ratio_lmax
        out["%s_%s_ratio_trace" % (tag, name)] = ratio_trace

    block_metrics(M, "full")

    if rot_idx is None or trans_idx is None:
        return
    if set(rot_idx) & set(trans_idx):
        sys.stderr.write(
            "  skipping block decomposition for '%s': rot_idx and trans_idx "
            "overlap (%s vs %s); Schur complements would be degenerate\n"
            % (tag, rot_idx, trans_idx))
        return
    n = M.shape[1]
    if n < max(max(rot_idx), max(trans_idx)) + 1:
        sys.stderr.write("  skipping block decomposition for '%s': matrix is "
                         "%dx%d, indices out of range\n" % (tag, n, n))
        return

    ri = np.ix_(list(rot_idx), list(rot_idx))
    ti = np.ix_(list(trans_idx), list(trans_idx))
    R = np.stack([M[i][ri] for i in range(rows)])
    T = np.stack([M[i][ti] for i in range(rows)])
    block_metrics(R, "rot")
    block_metrics(T, "trans")

    # Schur complements: the conditioning of one block AFTER accounting for
    # its coupling to the other. 
    rt = np.ix_(list(rot_idx), list(trans_idx))
    S_t = np.full_like(T, np.nan)
    S_r = np.full_like(R, np.nan)
    for i in range(rows):
        A, B, D = M[i][ri], M[i][rt], M[i][ti]
        if not (np.all(np.isfinite(A)) and np.all(np.isfinite(B))
                and np.all(np.isfinite(D))):
            continue
        try:
            S_t[i] = D - B.T @ np.linalg.pinv(A) @ B
            S_r[i] = A - B @ np.linalg.pinv(D) @ B.T
        except np.linalg.LinAlgError:
            continue
    block_metrics(S_t, "trans_schur")
    block_metrics(S_r, "rot_schur")


def normalised_conditioning(out, tag, n_corr_key):
    """Correspondence-count normalisation. 
    """
    n_corr = out.get(n_corr_key)
    if n_corr is None:
        return
    nc = np.asarray(n_corr, dtype=float)
    for name in ("full", "rot", "trans", "trans_schur", "rot_schur"):
        lmin_k = "%s_%s_lambda_min" % (tag, name)
        if lmin_k not in out:
            continue
        lmin = np.asarray(out[lmin_k], dtype=float)
        tr = np.asarray(out["%s_%s_trace" % (tag, name)], dtype=float)
        with np.errstate(divide="ignore", invalid="ignore"):
            out["%s_%s_lambda_min_per_corr" % (tag, name)] = np.where(
                nc > 0, lmin / nc, np.nan)
            out["%s_%s_trace_per_corr" % (tag, name)] = np.where(
                nc > 0, tr / nc, np.nan)


# ---------------------------------------------------------------------------
# Main per-run processing
# ---------------------------------------------------------------------------

def process_run(run_dir, tol=0.25, n_state=6, verbose=True):
    if verbose:
        sys.stderr.write("processing %s\n" % run_dir)
    feat, cond = load_run(run_dir)
    df = merge_on_time(cond, feat, tol)

    out = {"t": df["t"].to_numpy(dtype=float)}
    for c in ("t_feat", "join_dt"):
        if c in df.columns:
            out[c] = df[c].to_numpy(dtype=float)

    # Pass through raw scalars unchanged.
    passthrough = [c for c in df.columns
                   if not any(c.startswith(p) for p in
                              ("bear_", "elev_", "rng_", "scat_", "hessian_",
                               "AtA_", "surfAtA_", "cornerAtA_"))
                   and c not in ("t", "t_feat", "join_dt")]
    for c in passthrough:
        out[c] = df[c].to_numpy()

    # Histograms.
    for pfx in ("bear_corner", "bear_surf", "bear_all"):
        hist_metrics(df, pfx, out, circular=True)
    for pfx in ("elev_corner", "elev_surf", "elev_all"):
        hist_metrics(df, pfx, out, circular=False)
    for pfx in ("rng_corner", "rng_surf", "rng_all"):
        hist_metrics(df, pfx, out, circular=False)

    # Scatter moments -> shape descriptors.
    for pfx in ("scat_corner", "scat_surf", "scat_all"):
        cols = ["%s_%02d" % (pfx, i) for i in range(9)]
        if not all(c in df.columns for c in cols):
            continue
        V = df[cols].to_numpy(dtype=float)
        rows = V.shape[0]
        lam = np.full((rows, 3), np.nan)
        for i in range(rows):
            cxx, cxy, cxz, cyy, cyz, czz = V[i, 3:9]
            C = np.array([[cxx, cxy, cxz],
                          [cxy, cyy, cyz],
                          [cxz, cyz, czz]], dtype=float)
            if not np.all(np.isfinite(C)):
                continue
            try:
                w = np.sort(np.linalg.eigvalsh(C))[::-1]
            except np.linalg.LinAlgError:
                continue
            lam[i] = w
        l1, l2, l3 = lam[:, 0], lam[:, 1], lam[:, 2]
        with np.errstate(divide="ignore", invalid="ignore"):
            out[pfx + "_linearity"] = np.where(l1 > 0, (l1 - l2) / l1, np.nan)
            out[pfx + "_planarity"] = np.where(l1 > 0, (l2 - l3) / l1, np.nan)
            out[pfx + "_sphericity"] = np.where(l1 > 0, l3 / l1, np.nan)
        out[pfx + "_lambda1"] = l1
        out[pfx + "_lambda2"] = l2
        out[pfx + "_lambda3"] = l3

    # Information matrices.
    M6 = read_sym_matrix(df, "hessian", n_state)
    if M6 is None:
        M6 = read_full_matrix(df, "hessian", n_state)
    if M6 is not None:
        conditioning_metrics(M6, out, "map",
                             rot_idx=(0, 1, 2), trans_idx=(3, 4, 5))
        normalised_conditioning(out, "map", "n_corr")

    for pfx, tag in (("surfAtA", "surf"), ("cornerAtA", "corner")):
        M3 = read_sym_matrix(df, pfx, 3)
        if M3 is None:
            M3 = read_full_matrix(df, pfx, 3)
        if M3 is not None:
            conditioning_metrics(M3, out, tag)

    res = pd.DataFrame(out)
    res = res.loc[:, ~res.columns.duplicated()]
    dest = os.path.join(run_dir, "structure.csv")
    res.to_csv(dest, index=False)
    if verbose:
        sys.stderr.write("  wrote %s  (%d rows, %d cols)\n"
                         % (dest, len(res), res.shape[1]))
    return res


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("path", help="run directory, or parent directory with --batch")
    ap.add_argument("--batch", action="store_true",
                    help="treat path as a parent containing many run directories")
    ap.add_argument("--tol", type=float, default=0.25,
                    help="max seconds between a conditioning row and its feature row")
    ap.add_argument("--n-state", type=int, default=6,
                    help="dimension of the mapping-node state (default 6)")
    args = ap.parse_args()

    if args.batch:
        entries = sorted(
            os.path.join(args.path, d) for d in os.listdir(args.path)
            if os.path.isdir(os.path.join(args.path, d)))
        ok = 0
        for d in entries:
            if not (os.path.exists(os.path.join(d, "features.csv"))
                    or os.path.exists(os.path.join(d, "conditioning.csv"))):
                continue
            try:
                process_run(d, tol=args.tol, n_state=args.n_state)
                ok += 1
            except Exception as e:
                sys.stderr.write("  FAILED %s: %s\n" % (d, e))
        sys.stderr.write("done: %d run(s) processed\n" % ok)
    else:
        process_run(args.path, tol=args.tol, n_state=args.n_state)


if __name__ == "__main__":
    main()
