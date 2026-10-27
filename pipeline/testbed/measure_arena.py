#!/usr/bin/env python3
"""
Measure the testbed's nested structure from a LeGO-LOAM map PCD, and emit crop
box definitions for the ablation levels.

Finds, in order:
  1. the sand/ground plane, and levels the cloud against it
  2. the arena's yaw, since LeGO-LOAM's map frame is aligned to wherever the
     sensor happened to start, not to the building
  3. wall positions, from peaks in the x and y marginals once levelled and
     de-rotated
  4. the retaining wall's height above the sand

Reports everything in metres and prints ready-to-use --box arguments.

Usage:
    python3 measure_arena.py map.pcd
    python3 measure_arena.py map.pcd --wall-min-height 0.15 --plot arena.png

Requires: numpy (matplotlib only for --plot)
"""

import argparse
import re
import struct
import sys

import numpy as np

np.seterr(all='ignore')

PCD_TYPES = {
    ('F', 4): np.float32, ('F', 8): np.float64,
    ('U', 1): np.uint8,   ('U', 2): np.uint16, ('U', 4): np.uint32,
    ('I', 1): np.int8,    ('I', 2): np.int16,  ('I', 4): np.int32,
}


# ---------------------------------------------------------------- pcd reader

def read_pcd(path):
    """Read an ascii, binary or binary_compressed PCD into a dict of arrays."""
    with open(path, 'rb') as fh:
        header, lines = {}, []
        while True:
            line = fh.readline()
            if not line:
                sys.exit('unexpected end of header')
            txt = line.decode('ascii', 'replace').strip()
            lines.append(txt)
            if txt.startswith('#') or not txt:
                continue
            key, _, val = txt.partition(' ')
            header[key.upper()] = val.strip()
            if key.upper() == 'DATA':
                break

        fields = header['FIELDS'].split()
        sizes = [int(v) for v in header['SIZE'].split()]
        types = header['TYPE'].split()
        counts = [int(v) for v in header.get('COUNT',
                                             ' '.join(['1'] * len(fields))).split()]
        npoints = int(header['POINTS'])
        data_kind = header['DATA'].lower()

        names, formats = [], []
        for f, t, s, c in zip(fields, types, sizes, counts):
            if c != 1:
                for i in range(c):
                    names.append('%s_%d' % (f, i))
                    formats.append(PCD_TYPES[(t, s)])
            else:
                names.append(f)
                formats.append(PCD_TYPES[(t, s)])
        dtype = np.dtype(list(zip(names, formats)))

        if data_kind == 'ascii':
            raw = np.loadtxt(fh, dtype=np.float64, ndmin=2)
            out = {n: raw[:, i] for i, n in enumerate(names)}
        elif data_kind == 'binary':
            buf = fh.read(dtype.itemsize * npoints)
            arr = np.frombuffer(buf, dtype=dtype, count=npoints)
            out = {n: arr[n].astype(np.float64) for n in names}
        elif data_kind == 'binary_compressed':
            comp, uncomp = struct.unpack('<II', fh.read(8))
            import lzf  # noqa
            blob = lzf.decompress(fh.read(comp), uncomp)
            out, off = {}, 0
            for n, f in zip(names, formats):
                w = np.dtype(f).itemsize * npoints
                out[n] = np.frombuffer(blob[off:off + w],
                                       dtype=f).astype(np.float64)
                off += w
        else:
            sys.exit('unsupported DATA %s' % data_kind)

    for a in 'xyz':
        if a not in out:
            sys.exit('PCD has no %s field (has: %s)' % (a, ', '.join(names)))
    p = np.column_stack([out['x'], out['y'], out['z']])
    return p[np.isfinite(p).all(axis=1)]


# ---------------------------------------------------------------- geometry

def fit_plane(points, thresh=0.05, iters=800, seed=0):
    rng = np.random.default_rng(seed)
    best_n, best_p, best = None, None, 0
    for _ in range(iters):
        i = rng.choice(len(points), 3, replace=False)
        p0, p1, p2 = points[i]
        n = np.cross(p1 - p0, p2 - p0)
        nn = np.linalg.norm(n)
        if nn < 1e-9:
            continue
        n = n / nn
        c = int((np.abs((points - p0) @ n) < thresh).sum())
        if c > best:
            best_n, best_p, best = n, p0, c
    inl = points[np.abs((points - best_p) @ best_n) < thresh]
    centre = inl.mean(axis=0)
    n = np.linalg.svd(inl - centre, full_matrices=False)[2][2]
    if n[2] < 0:
        n = -n
    return n, centre, len(inl)


def level_rotation(n):
    z = np.array([0.0, 0.0, 1.0])
    v = np.cross(n, z)
    s = np.linalg.norm(v)
    if s < 1e-9:
        return np.eye(3)
    c = float(n @ z)
    vx = np.array([[0, -v[2], v[1]], [v[2], 0, -v[0]], [-v[1], v[0], 0]])
    return np.eye(3) + vx + vx @ vx * ((1 - c) / s ** 2)


def find_yaw(xy, step=0.25):
    """Angle at which the walls line up with the axes.

    Rotating the cloud and scoring how concentrated the x and y marginals are:
    walls parallel to an axis pile into a few bins, walls at an angle smear
    across many.
    """
    best_a, best_s = 0.0, -1.0
    for a in np.arange(0.0, 90.0, step):
        r = np.radians(a)
        c, s = np.cos(r), np.sin(r)
        u = xy[:, 0] * c + xy[:, 1] * s
        v = -xy[:, 0] * s + xy[:, 1] * c
        score = 0.0
        for arr in (u, v):
            h, _ = np.histogram(arr, bins=np.arange(arr.min(), arr.max() + 0.1, 0.1))
            score += float(np.sort(h)[-20:].sum())
        if score > best_s:
            best_s, best_a = score, a
    return best_a


def wall_peaks(vals, min_sep=1.0, top=6):
    h, edges = np.histogram(vals, bins=np.arange(vals.min(), vals.max() + 0.1, 0.1))
    centres = 0.5 * (edges[:-1] + edges[1:])
    order = np.argsort(-h)
    picked = []
    for i in order:
        if h[i] < 0.15 * h.max():
            break
        if all(abs(centres[i] - p) > min_sep for p, _ in picked):
            picked.append((centres[i], int(h[i])))
        if len(picked) >= top:
            break
    return sorted(picked)


# ---------------------------------------------------------------- main

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('pcd')
    ap.add_argument('--wall-min-height', type=float, default=0.15,
                    help='height above sand at which to start looking for walls')
    ap.add_argument('--wall-max-height', type=float, default=3.0)
    ap.add_argument('--plot', default=None, help='write a PNG of the fits')
    args = ap.parse_args()

    p = read_pcd(args.pcd)
    print('points loaded : %d' % len(p))

    # --- sand plane ---
    lowish = p[p[:, 2] < np.percentile(p[:, 2], 40)]
    n, centre, ninl = fit_plane(lowish)
    R = level_rotation(n)
    L = p @ R.T
    ground_z = float(np.median((centre @ R.T)[2]) if centre.ndim else 0.0)
    ground_z = float(np.percentile(L[:, 2], 2))
    H = L[:, 2] - ground_z

    print('sand plane    : normal [%+.3f %+.3f %+.3f], %d inliers'
          % (n[0], n[1], n[2], ninl))
    print('               tilt %.2f deg from level'
          % np.degrees(np.arccos(abs(n[2]))))
    print('')

    # --- height structure ---
    print('height distribution above sand:')
    for q in (50, 75, 90, 95, 98, 99, 100):
        print('  p%-3d : %6.2f m' % (q, np.percentile(H, q)))
    print('')

    # --- arena yaw ---
    band = (H > args.wall_min_height) & (H < args.wall_max_height)
    xy = L[band, :2]
    if len(xy) < 500:
        sys.exit('too few wall-height points (%d); lower --wall-min-height' % len(xy))
    yaw = find_yaw(xy)
    r = np.radians(yaw)
    c, s = np.cos(r), np.sin(r)
    U = L[:, 0] * c + L[:, 1] * s
    V = -L[:, 0] * s + L[:, 1] * c
    print('arena yaw in map frame : %.2f deg' % yaw)
    print('  (LeGO-LOAM aligns its map to the sensor start pose, not the room)')
    print('')

    # --- wall positions ---
    for axis, vals in (('u', U[band]), ('v', V[band])):
        peaks = wall_peaks(vals)
        print('%s-axis wall lines:' % axis)
        for pos, cnt in peaks:
            print('   %+7.2f m   (%d points)' % (pos, cnt))
        if len(peaks) >= 2:
            print('   outermost span : %.2f m' % (peaks[-1][0] - peaks[0][0]))
        print('')

    # --- retaining wall height ---
    print('retaining wall height:')
    print('  pick the INNER pair of lines above, then re-run with --plot to')
    print('  confirm. Height profile of points near each candidate line:')
    for axis, vals, other in (('u', U, V), ('v', V, U)):
        peaks = wall_peaks(vals[band])
        for pos, _ in peaks:
            near = np.abs(vals - pos) < 0.15
            if near.sum() < 200:
                continue
            hh = H[near]
            print('   %s=%+6.2f : %6d pts, height p50 %.2f p90 %.2f p99 %.2f max %.2f'
                  % (axis, pos, near.sum(), np.percentile(hh, 50),
                     np.percentile(hh, 90), np.percentile(hh, 99), hh.max()))
    print('')
    print('  A retaining wall tops out well below the outer walls. Look for a')
    print('  line whose p99 is a few tenths of a metre while others run metres.')
    print('')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(21, 7))
        for ax, (lo, hi) in zip(axs, [(0.02, 0.20), (0.20, 0.60), (0.60, 3.0)]):
            m = (H >= lo) & (H < hi)
            ax.scatter(U[m], V[m], s=0.08, c='k', alpha=0.25)
            ax.set_aspect('equal')
            ax.grid(alpha=0.3)
            ax.set_title('%.2f - %.2f m above sand  (%d pts)' % (lo, hi, m.sum()))
            ax.set_xlabel('u [m]')
            ax.set_ylabel('v [m]')
        fig.tight_layout()
        fig.savefig(args.plot, dpi=100)
        print('wrote %s' % args.plot)

    print('=' * 64)
    print('Next: choose boundaries from the lines above and build the levels.')
    print('crop_map_frame.py takes an axis-aligned box in the MAP frame, so if')
    print('the yaw above is not near 0 the box will not fit the arena -- tell')
    print('me the yaw and I will add rotation support.')
    print('=' * 64)


if __name__ == '__main__':
    main()
