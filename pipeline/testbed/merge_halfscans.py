#!/usr/bin/env python3
"""
Merge alternating half-revolution PointCloud2 messages into full-revolution scans.

This merges each consecutive pair into one full-revolution unorganised cloud and
writes a new ROS 1 bag at 10 Hz.

Usage:
    python3 merge_halfscans.py testbed_1.bag testbed_1_merged.bag
    python3 merge_halfscans.py testbed_1.bag testbed_1_merged.bag --keep-dual
    python3 merge_halfscans.py testbed_1.bag testbed_1_merged.bag --keep-nan

Requires:  pip install rosbags     (tested on 0.9.23 / Python 3.8)

Operates on the ROS 1 serialised bytes directly, so it does not depend on the
rosbags message-definition API and is unaffected by version differences.
"""

import argparse
import struct
import sys

import numpy as np
from rosbags.rosbag1 import Reader, Writer

DTYPES = {
    1: np.int8,   2: np.uint8,  3: np.int16,   4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def parse_pointcloud2(raw):
    """Parse a ROS 1 serialised sensor_msgs/PointCloud2 into its parts."""
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

    def string():
        nonlocal p
        n = u32()
        v = bytes(raw[p:p + n])
        p += n
        return v

    head_start = p
    u32()                       # header.seq
    u32()                       # header.stamp.secs
    u32()                       # header.stamp.nsecs
    string()                    # header.frame_id
    head = bytes(raw[head_start:p])

    height = u32()
    width = u32()

    p_fields = p
    nfields = u32()
    fields = []
    for _ in range(nfields):
        name = string()
        offset = u32()
        datatype = u8()
        count = u32()
        fields.append((name, offset, datatype, count))
    fields_blob = bytes(raw[p_fields:p])

    u8()                        # is_bigendian
    point_step = u32()
    u32()                       # row_step
    data_len = u32()
    data = raw[p:p + data_len]
    p += data_len
    u8()                        # is_dense

    return {
        'head': head,
        'height': height,
        'width': width,
        'fields_blob': fields_blob,
        'fields': fields,
        'point_step': point_step,
        'data': data,
    }


def build_pointcloud2(msg, points_bytes, npoints, is_dense):
    """Reassemble a ROS 1 serialised PointCloud2 as an unorganised cloud."""
    step = msg['point_step']
    out = bytearray()
    out += msg['head']
    out += struct.pack('<II', 1, npoints)              # height=1, width=npoints
    out += msg['fields_blob']
    out += struct.pack('<B', 0)                        # is_bigendian
    out += struct.pack('<II', step, step * npoints)    # point_step, row_step
    out += struct.pack('<I', len(points_bytes))
    out += points_bytes
    out += struct.pack('<B', 1 if is_dense else 0)     # is_dense
    return bytes(out)


def struct_dtype(msg):
    names, formats, offsets = [], [], []
    for name, offset, datatype, _count in msg['fields']:
        names.append(name.decode('ascii'))
        formats.append(DTYPES[datatype])
        offsets.append(offset)
    return np.dtype({
        'names': names,
        'formats': formats,
        'offsets': offsets,
        'itemsize': msg['point_step'],
    })


def looks_dual_return(arr2d):
    """True if rows pair up as duplicate returns at the same azimuth."""
    if arr2d.shape[0] % 2:
        return False
    a, b = arr2d[0::2], arr2d[1::2]
    ra = np.sqrt(a['x'].astype(np.float64) ** 2 + a['y'].astype(np.float64) ** 2)
    rb = np.sqrt(b['x'].astype(np.float64) ** 2 + b['y'].astype(np.float64) ** 2)
    both = np.isfinite(ra) & np.isfinite(rb)
    if both.sum() < 100:
        return False
    return float((np.abs(ra[both] - rb[both]) < 1e-4).mean()) > 0.90


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('src')
    ap.add_argument('dst')
    ap.add_argument('--topic', default='/velodyne_points')
    ap.add_argument('--keep-nan', action='store_true',
                    help='keep invalid (no-return) points')
    ap.add_argument('--keep-dual', action='store_true',
                    help='keep both returns instead of dropping the duplicate')
    args = ap.parse_args()

    pending = None
    seen = written = in_pts = out_pts = 0
    dual = None

    with Reader(args.src) as reader:
        conns = [c for c in reader.connections if c.topic == args.topic]
        if not conns:
            avail = sorted({c.topic for c in reader.connections})
            sys.exit('Topic %s not in bag. Available: %s' % (args.topic, avail))
        src = conns[0]
        msgtype = src.msgtype

        with Writer(args.dst) as writer:
            # Carry the source connection's message definition and md5sum across.
            # Without them the writer derives an md5sum that ROS 1 subscribers
            # reject ("wants topic ... to have datatype/md5sum ...").
            try:
                out_conn = writer.add_connection(
                    args.topic, msgtype,
                    msgdef=src.msgdef, md5sum=src.digest,
                )
            except TypeError:
                out_conn = writer.add_connection(args.topic, msgtype)
                print('warning: this rosbags version does not accept msgdef/md5sum;'
                      ' subscribers may reject the output bag')

            for _conn, timestamp, raw in reader.messages(connections=conns):
                msg = parse_pointcloud2(raw)
                dtype = struct_dtype(msg)
                h, w = msg['height'], msg['width']
                arr = np.frombuffer(bytes(msg['data']), dtype=dtype, count=h * w)
                in_pts += h * w

                grid = arr.reshape(h, w)
                if dual is None:
                    dual = looks_dual_return(grid)
                    print('dual-return detected: %s%s'
                          % (dual, ' (keeping both)' if args.keep_dual else ''))
                if dual and not args.keep_dual:
                    arr = grid[0::2].reshape(-1)
                else:
                    arr = grid.reshape(-1)

                if not args.keep_nan:
                    arr = arr[np.isfinite(arr['x'])
                              & np.isfinite(arr['y'])
                              & np.isfinite(arr['z'])]

                seen += 1
                if pending is None:
                    pending = arr
                    continue

                combined = np.concatenate([pending, arr])
                writer.write(out_conn, timestamp,
                             build_pointcloud2(msg, combined.tobytes(),
                                               len(combined),
                                               is_dense=not args.keep_nan))
                out_pts += len(combined)
                written += 1
                pending = None

    print('read   %d messages (%d points)' % (seen, in_pts))
    print('wrote  %d merged scans (%d points) -> %s' % (written, out_pts, args.dst))
    if written:
        print('avg    %d points per full-revolution scan' % (out_pts // written))
    if seen % 2:
        print('note:  odd message count, trailing half-scan dropped')


if __name__ == '__main__':
    main()
