#!/usr/bin/env python3
"""
ceiling_audit.py --

reprocess_lim.sh only compares the first ~30 clouds of each bag, which is the
opening parked window. This compares every cloud.

crop_map_frame.py writes the ORIGINAL sensor-frame points it keeps, byte for
byte, so a point missing from the ceilinged bag is exactly a point the ceiling
removed. No tf is needed: clouds are paired by header stamp and points are
matched by their exact x, y, z.

The removed points are then broken down by range from the sensor. Overhead
building structure sits 13-18 m out. Anything removed within ~10 m is inside
the pit and is terrain -- rock tops, mound crests -- which the ceiling should
not be taking.

Usage:
    python3 ceiling_audit.py zigzag_level3.bag zigzag_lim_level3.bag
    python3 ceiling_audit.py level3.bag september_lim_level3.bag --near 10
"""

import argparse
import sys

import numpy as np
import rosbag

DTYPES = {
    1: np.int8, 2: np.uint8, 3: np.int16, 4: np.uint16,
    5: np.int32, 6: np.uint32, 7: np.float32, 8: np.float64,
}


def xyz(msg):
    """(n, 3) float32 array of the cloud's x, y, z, NaNs dropped."""
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(DTYPES[f.datatype])
        offsets.append(f.offset)
    dt = np.dtype({'names': names, 'formats': formats,
                   'offsets': offsets, 'itemsize': msg.point_step})
    arr = np.frombuffer(msg.data, dtype=dt, count=msg.width * msg.height)
    p = np.column_stack([arr['x'], arr['y'], arr['z']]).astype(np.float32)
    return p[np.isfinite(p).all(axis=1)]


def clouds(path):
    """{stamp_ns: (n, 3) float32} for every PointCloud2 in the bag."""
    out = {}
    with rosbag.Bag(path) as b:
        for _topic, m, _t in b.read_messages():
            if m._type != 'sensor_msgs/PointCloud2':
                continue
            out[m.header.stamp.to_nsec()] = xyz(m)
    return out


def as_keys(p):
    """Exact-bit keys for each point, so membership is an exact match."""
    u = np.ascontiguousarray(p).view(np.uint32).reshape(-1, 3)
    return u.view([('x', 'u4'), ('y', 'u4'), ('z', 'u4')]).ravel()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('original', help='level-3 bag WITHOUT the ceiling')
    ap.add_argument('ceilinged', help='the _lim level-3 bag')
    ap.add_argument('--near', type=float, default=10.0,
                    help='range [m] inside which a removed point counts as '
                         'terrain rather than overhead structure (default 10)')
    args = ap.parse_args()

    print('reading %s' % args.original)
    a = clouds(args.original)
    print('reading %s' % args.ceilinged)
    b = clouds(args.ceilinged)

    shared = sorted(set(a) & set(b))
    print('')
    print('clouds: original %d, ceilinged %d, paired by stamp %d'
          % (len(a), len(b), len(shared)))
    if not shared:
        sys.exit('no clouds share a stamp -- these are not the same recording')

    t0 = shared[0]
    bins = [0, 5, 10, 13, 18, 1e9]
    labels = ['0-5 m', '5-10 m', '10-13 m', '13-18 m', '>18 m']
    by_range = np.zeros(len(labels), dtype=np.int64)
    n_orig = n_removed = n_extra = 0
    per_scan = []       # (t, removed, removed_near)
    near_heights = []

    for s in shared:
        pa, pb = a[s], b[s]
        n_orig += len(pa)
        inb = np.isin(as_keys(pa), as_keys(pb))
        n_extra += max(0, len(pb) - int(inb.sum()))   # should be 0
        rem = pa[~inb]
        n_removed += len(rem)
        r = np.linalg.norm(rem, axis=1)
        by_range += np.histogram(r, bins=bins)[0]
        near = r < args.near
        if near.any():
            near_heights.append(rem[near, 2])
        per_scan.append(((s - t0) * 1e-9, len(rem), int(near.sum())))

    per_scan = np.array(per_scan)
    n = len(shared)

    print('')
    print('=' * 64)
    print('WHOLE RUN')
    print('=' * 64)
    print('points in original    : %d  (%.0f per cloud)' % (n_orig, n_orig / n))
    print('removed by the ceiling: %d  (%.3f%% of points, %.1f per cloud)'
          % (n_removed, 100.0 * n_removed / max(n_orig, 1), n_removed / n))
    if n_extra:
        print('  !! %d points in the ceilinged bag are NOT in the original --'
              % n_extra)
        print('     the two bags were not cropped from the same input')

    print('')
    print('removed points by range from the sensor:')
    for lab, c in zip(labels, by_range):
        print('   %-8s %9d   %5.1f%%'
              % (lab, c, 100.0 * c / max(n_removed, 1)))

    print('')
    print('through the run (tenths of the recording):')
    print('   %8s %12s %18s' % ('t [s]', 'removed/cloud', 'within %.0f m/cloud'
                                % args.near))
    edges = np.linspace(per_scan[0, 0], per_scan[-1, 0], 11)
    for lo, hi in zip(edges[:-1], edges[1:]):
        m = (per_scan[:, 0] >= lo) & (per_scan[:, 0] <= hi)
        if m.any():
            print('   %8.1f %12.1f %18.1f'
                  % (lo, per_scan[m, 1].mean(), per_scan[m, 2].mean()))

    near_total = int(per_scan[:, 2].sum())
    print('')
    print('=' * 64)
    print('VERDICT')
    print('=' * 64)
    if near_total == 0:
        print('Nothing within %.0f m was removed at any point in the run. The'
              % args.near)
        print('ceiling took overhead structure only; level 3 is not missing')
        print('any terrain.')
    else:
        h = np.concatenate(near_heights)
        print('%d points within %.0f m were removed (%.2f per cloud, %.4f%% of'
              % (near_total, args.near, near_total / n,
                 100.0 * near_total / max(n_orig, 1)))
        print('all points). Their height in the sensor frame: p50 %+.2f m,'
              % np.percentile(h, 50))
        print('p90 %+.2f m, max %+.2f m. The sensor is 0.652 m above the sand,'
              % (np.percentile(h, 90), h.max()))
        print('so points near -0.4 m are ~0.25 m above the sand: rock tops or')
        print('mound crests. Check WHEN in the run they were removed (table')
        print('above): a block of them while the rover is on low ground means')
        print('the map-frame ceiling cut terrain that sits higher than the start.')
        print('')
        print('Whether it matters depends on the count: a few points per cloud')
        print('out of ~20,000 is negligible for lambda_min; hundreds is not.')


if __name__ == '__main__':
    main()
