#!/usr/bin/env python3
"""
Compare ablation levels scan by scan.

Reports, per level, the quantities that define the manipulation dose and the
geometric conditioning that follows from it:

  points/scan     how much was kept
  azimuth cover   how much of the horizon still returns anything
  range spread    how far the remaining structure sits
  normal spread   how many distinct surface orientations survive

LOAM-family scan matching constrains translation through planar features, and the information available is

    H = sum over features of n n^T

with n the surface normal. Its smallest eigenvalue is the weakest constrained
direction: strip the walls and every remaining normal points up, H collapses
toward rank 1, and horizontal translation stops being observable. Reported
here normalised by feature count, so it measures geometry rather than quantity.

Usage:
    python3 compare_levels.py level0.bag level1.bag level2.bag level3.bag
    python3 compare_levels.py *.bag --every 10 --plot levels.png

Requires: numpy, scipy, rosbags
"""

import argparse
import struct
import sys

import numpy as np
from rosbags.rosbag1 import Reader

np.seterr(all='ignore')

DTYPES = {
    1: np.int8,   2: np.uint8,  3: np.int16,   4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def parse_pointcloud2(raw):
    p = 0

    def u32():
        nonlocal p
        v = struct.unpack_from('<I', raw, p)[0]
        p += 4
        return v

    def u8():
        nonlocal p
        v = raw[p]
        p += 1
        return v

    def s():
        nonlocal p
        n = u32()
        v = raw[p:p + n]
        p += n
        return v

    u32(); u32(); u32(); s()
    h = u32(); w = u32(); nf = u32()
    fields = []
    for _ in range(nf):
        nm = s().decode('ascii', 'replace')
        off = u32()
        dt = u8()
        u32()
        fields.append((nm, off, dt))
    u8()
    ps = u32(); u32(); dl = u32()
    blob = raw[p:p + dl]
    dtype = np.dtype({'names': [f[0] for f in fields],
                      'formats': [DTYPES[f[2]] for f in fields],
                      'offsets': [f[1] for f in fields],
                      'itemsize': ps})
    return np.frombuffer(blob, dtype=dtype, count=min(h * w, len(blob) // ps))


def normal_conditioning(pts, k=12, sample=4000, seed=0):
    """lambda_min of the normalised normal information matrix, and its
    condition number. Returns (lambda_min, cond, n_used)."""
    try:
        from scipy.spatial import cKDTree
    except ImportError:
        sys.exit('needs scipy: pip3 install --user scipy')

    if len(pts) < k * 4:
        return np.nan, np.nan, 0
    rng = np.random.default_rng(seed)
    tree = cKDTree(pts)
    idx = rng.choice(len(pts), min(sample, len(pts)), replace=False)
    _, nb = tree.query(pts[idx], k=k)

    nbrs = pts[nb]                               # (m, k, 3)
    centred = nbrs - nbrs.mean(axis=1, keepdims=True)
    cov = np.einsum('mki,mkj->mij', centred, centred) / k
    evals, evecs = np.linalg.eigh(cov)

    # planar patches only: smallest eigenvalue well below the next
    planar = evals[:, 0] < 0.15 * evals[:, 1]
    if planar.sum() < 50:
        return np.nan, np.nan, int(planar.sum())

    n = evecs[planar, :, 0]                      # normal = smallest-variance axis
    n = n / np.linalg.norm(n, axis=1, keepdims=True)
    H = n.T.dot(n) / len(n)                      # normalised: geometry, not count
    w = np.linalg.eigvalsh(H)
    return float(w[0]), float(w[2] / max(w[0], 1e-12)), int(planar.sum())


def scan_stats(arr, every_pt=4):
    x = arr['x'].astype(np.float64)
    y = arr['y'].astype(np.float64)
    z = arr['z'].astype(np.float64)
    ok = np.isfinite(x) & np.isfinite(y) & np.isfinite(z)
    x, y, z = x[ok], y[ok], z[ok]
    if len(x) < 100:
        return None
    az = np.degrees(np.arctan2(y, x))
    r = np.hypot(x, y)
    h, _ = np.histogram(az, bins=72, range=(-180, 180))
    pts = np.column_stack([x, y, z])[::every_pt]
    lmin, cond, npl = normal_conditioning(pts)
    return dict(n=len(x), cover=int((h > 5).sum()),
                r50=float(np.percentile(r, 50)),
                r95=float(np.percentile(r, 95)),
                zspread=float(np.percentile(z, 95) - np.percentile(z, 5)),
                lmin=lmin, cond=cond, nplanar=npl)


def analyse(path, topic, every):
    rows = []
    with Reader(path) as rd:
        conns = [c for c in rd.connections if topic is None or c.topic == topic]
        if not conns:
            avail = sorted({c.topic for c in rd.connections})
            sys.exit('no matching topic in %s (has: %s)' % (path, avail))
        for i, (_c, _t, raw) in enumerate(rd.messages(connections=conns)):
            if i % every:
                continue
            st = scan_stats(parse_pointcloud2(raw))
            if st:
                rows.append(st)
    return rows


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--topic', default=None,
                    help='default: whichever point cloud topic the bag has')
    ap.add_argument('--every', type=int, default=10,
                    help='analyse every Nth scan (default 10)')
    ap.add_argument('--plot', default=None)
    args = ap.parse_args()

    results = {}
    for b in args.bags:
        name = b.rsplit('/', 1)[-1].replace('.bag', '')
        sys.stderr.write('reading %s ...\n' % name)
        sys.stderr.flush()
        results[name] = analyse(b, args.topic, args.every)

    hdr = ('%-12s %9s %8s %8s %8s %9s %9s %8s'
           % ('level', 'pts/scan', 'cover', 'r50', 'r95', 'lambda', 'cond', 'planar'))
    print('')
    print(hdr)
    print('-' * len(hdr))
    for name, rows in results.items():
        if not rows:
            print('%-12s  no usable scans' % name)
            continue
        g = lambda k: np.array([r[k] for r in rows], dtype=float)
        print('%-12s %9d %8.1f %8.2f %8.2f %9.4f %9.1f %8d'
              % (name, np.median(g('n')), np.median(g('cover')),
                 np.median(g('r50')), np.median(g('r95')),
                 np.nanmedian(g('lmin')), np.nanmedian(g('cond')),
                 np.median(g('nplanar'))))

    print('')
    print('cover  : azimuth sectors with returns, of 72')
    print('lambda : smallest eigenvalue of the normalised normal matrix.')
    print('         higher = surfaces face more directions = translation')
    print('         better constrained. Should FALL as levels strip walls.')
    print('cond   : largest/smallest eigenvalue. Should RISE.')
    print('')

    names = list(results)
    if len(names) > 1:
        base = names[0]
        b0 = np.nanmedian([r['lmin'] for r in results[base]])
        print('relative to %s:' % base)
        for n in names[1:]:
            v = np.nanmedian([r['lmin'] for r in results[n]])
            print('  %-12s lambda x%.2f' % (n, v / b0))
        print('')
        vals = [np.nanmedian([r['lmin'] for r in results[n]]) for n in names]
        if all(np.diff(vals) < 0):
            print('Monotonic decrease across levels: the ablation is biting.')
        else:
            print('NOT monotonic. Check the box bounds -- a level that is not')
            print('strictly inside the previous one will break the dose axis.')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 3, figsize=(18, 5))
        for name, rows in results.items():
            t = np.arange(len(rows)) * args.every * 0.1
            axs[0].plot(t, [r['n'] for r in rows], lw=0.9, label=name)
            axs[1].plot(t, [r['lmin'] for r in rows], lw=0.9, label=name)
            axs[2].plot(t, [r['cover'] for r in rows], lw=0.9, label=name)
        for ax, ttl in zip(axs, ['points per scan',
                                 'lambda_min (normal matrix)',
                                 'azimuth sectors of 72']):
            ax.set_title(ttl)
            ax.set_xlabel('t [s]')
            ax.grid(alpha=0.3)
            ax.legend(fontsize=8)
        axs[0].set_yscale('log')
        fig.tight_layout()
        fig.savefig(args.plot, dpi=100)
        print('wrote %s' % args.plot)


if __name__ == '__main__':
    main()
