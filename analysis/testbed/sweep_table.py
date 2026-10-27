#!/usr/bin/env python3
"""
Collect the keep_fraction sweep into one table: dose (lambda_min) against
response (drift), plus the endpoint checks.

Reads each level's structure.csv for lambda_min and est_raw_*.txt for the
trajectory, and reports drift as divergence from the keep_fraction=1.00 run --
the same metric used for the box levels, since the physical recording's loop
does not close and return-to-origin is unusable.

Usage:
    python3 sweep_table.py results/testbed/sweep
    python3 sweep_table.py results/testbed/sweep --plot sweep.png
"""

import argparse
import glob
import os
import sys

import numpy as np

np.seterr(all='ignore')


def kf_of(name):
    """KF100 -> 1.00, KF035 -> 0.35"""
    digits = ''.join(c for c in name if c.isdigit())
    if not digits:
        return float('nan')
    return float(digits) / (10 ** (len(digits) - 1))


def load_traj(path):
    a = np.loadtxt(path, ndmin=2)
    return a[:, 0], a[:, 1:4]


def resample(t_src, p_src, t_dst):
    return np.column_stack([np.interp(t_dst, t_src, p_src[:, k])
                            for k in range(3)])


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('sweepdir')
    ap.add_argument('--column', default='map_full_lambda_min')
    ap.add_argument('--plot', default=None)
    args = ap.parse_args()

    try:
        import pandas as pd
    except ImportError:
        sys.exit('needs pandas: pip3 install --user pandas')

    rows = []
    for f in sorted(glob.glob(os.path.join(args.sweepdir, 'est_raw_*.txt'))):
        name = os.path.basename(f).replace('est_raw_', '').replace('.txt', '')
        s_path = os.path.join(args.sweepdir, 'logs', name, 'structure.csv')
        lam = n_corr = degen = float('nan')
        if os.path.exists(s_path):
            s = pd.read_csv(s_path)
            if 'map_ran' in s:
                s = s[s['map_ran'] == 1]
            lam = s[args.column].median()
            n_corr = s['n_corr'].median()
            degen = 100 * s['is_degenerate'].mean()
        t, p = load_traj(f)
        rows.append(dict(name=name, kf=kf_of(name), lam=lam, n_corr=n_corr,
                         degen=degen, t=t, p=p,
                         plen=float(np.sum(np.linalg.norm(np.diff(p, axis=0),
                                                          axis=1)))))
    if not rows:
        sys.exit('no est_raw_*.txt in %s' % args.sweepdir)

    rows.sort(key=lambda r: -r['kf'])
    base = rows[0]
    for r in rows:
        if r is base:
            r['div'] = 0.0
            continue
        lo, hi = max(r['t'][0], base['t'][0]), min(r['t'][-1], base['t'][-1])
        m = (r['t'] >= lo) & (r['t'] <= hi)
        r['div'] = (float(np.median(np.linalg.norm(
            r['p'][m] - resample(base['t'], base['p'], r['t'][m]), axis=1)))
            if m.sum() > 5 else float('nan'))

    hdr = ('%-8s %6s %10s %8s %8s %11s %9s'
           % ('run', 'keep', 'lambda_min', 'n_corr', 'degen%', 'divergence',
              'path len'))
    print('')
    print(hdr)
    print('-' * len(hdr))
    for r in rows:
        print('%-8s %6.2f %10.2f %8.0f %7.1f%% %10.3fm %8.2fm'
              % (r['name'], r['kf'], r['lam'], r['n_corr'], r['degen'],
                 r['div'], r['plen']))

    print('')
    print('divergence is median distance from %s at matched times.' % base['name'])
    print('')

    lams = [r['lam'] for r in rows if np.isfinite(r['lam'])]
    if len(lams) > 2:
        if all(np.diff(lams) < 0):
            print('lambda_min falls monotonically with keep_fraction -- the')
            print('dial is behaving as a dose axis.')
        else:
            print('lambda_min is NOT monotonic in keep_fraction. Check whether')
            print('plane detection is finding the same walls at every level')
            print('(_min_inliers may be cutting in at low retention).')
    print('')
    print('ENDPOINT CHECK -- compare against the box-crop levels:')
    print('  keep_fraction 1.00 should match L1   (lambda ~386)')
    print('  keep_fraction 0.00 should match L3   (lambda ~11)')
    print('  A mismatch means plane detection is not removing the same walls')
    print('  the box crop did, and the two sweeps are not on one axis.')

    if args.plot:
        import matplotlib
        matplotlib.use('Agg')
        import matplotlib.pyplot as plt
        fig, axs = plt.subplots(1, 2, figsize=(14, 5.5))
        kf = [r['kf'] for r in rows]
        axs[0].plot(kf, [r['lam'] for r in rows], 'o-')
        axs[0].set_yscale('log')
        axs[0].set_xlabel('keep_fraction')
        axs[0].set_ylabel('lambda_min')
        axs[0].set_title('dose: structure retained vs conditioning')
        axs[0].grid(alpha=0.3, which='both')
        good = [r for r in rows if np.isfinite(r['lam']) and r['div'] > 0]
        if good:
            axs[1].plot([r['lam'] for r in good], [r['div'] for r in good], 'o-')
            axs[1].set_xscale('log')
            axs[1].set_yscale('log')
        axs[1].set_xlabel('lambda_min')
        axs[1].set_ylabel('divergence from %s [m]' % base['name'])
        axs[1].set_title('response: conditioning vs drift')
        axs[1].grid(alpha=0.3, which='both')
        fig.tight_layout()
        fig.savefig(args.plot, dpi=100)
        print('')
        print('wrote %s' % args.plot)


if __name__ == '__main__':
    main()
