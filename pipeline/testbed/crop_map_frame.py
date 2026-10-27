#!/usr/bin/env python
"""
Crop LiDAR scans to a fixed box in the map frame.

Filtering against a volume fixed in the world means the manipulation is
identical at every rover pose, unlike a rover-relative cut which changes
meaning as the rover drives.

The box test is done in map coordinates, but the points written out are the
ORIGINAL rover-frame coordinates. Writing transformed coordinates would break
the correspondence between the `ring` field and elevation, which LeGO-LOAM's
range-image projection depends on.

Requires a tf tree, so run it while replaying the scan bag together with a tf
bag recorded during an earlier unfiltered pass.

  --dry-run     report poses only, publish nothing. Use this first.
  --box         xmin xmax ymin ymax zmin zmax, in map coordinates
  --yaw         degrees to rotate the box about the map origin. LeGO-LOAM
                aligns its map to the sensor's start pose, not to the room,
                so the arena is almost never axis-aligned. Take this from
                measure_arena.py and give the box in arena (u,v) coordinates.
  --stationary  start,end in seconds; report how far the pose moved over a still
                window, should be ~0.

Usage:
    rosrun <pkg> crop_map_frame.py --dry-run --stationary 0,6.5
    rosrun <pkg> crop_map_frame.py --box -3 3 -3 3 -2 1
"""

import argparse
import sys

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2
from tf.transformations import quaternion_matrix

DTYPES = {
    1: np.int8,   2: np.uint8,  3: np.int16,   4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def struct_dtype(msg):
    names, formats, offsets = [], [], []
    for f in msg.fields:
        names.append(f.name)
        formats.append(DTYPES[f.datatype])
        offsets.append(f.offset)
    return np.dtype({'names': names, 'formats': formats,
                     'offsets': offsets, 'itemsize': msg.point_step})


class Cropper(object):
    def __init__(self, args):
        self.args = args
        self.first_stamp = None
        self.poses = []          # (t_rel, x, y, z)
        self.n_in = 0
        self.n_kept = 0
        self.n_frames = 0
        self.n_failed = 0

        # a long cache so lookups near the start of a replay still resolve
        self.buf = tf2_ros.Buffer(rospy.Duration(3600.0))
        self.listener = tf2_ros.TransformListener(self.buf)

        self.pub = None
        if not args.dry_run:
            self.pub = rospy.Publisher(args.out_topic, PointCloud2, queue_size=5)

        rospy.Subscriber(args.in_topic, PointCloud2, self.cb, queue_size=5)
        rospy.on_shutdown(self.report)

    def lookup(self, stamp):
        """Transform at the scan's stamp, or None.
        """
        try:
            return self.buf.lookup_transform(
                self.args.target_frame, self.args.source_frame,
                stamp, rospy.Duration(self.args.tf_timeout))
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as e:
            self.n_failed += 1
            if self.n_failed <= 3:
                rospy.logwarn('tf lookup failed (timeout %.1fs): %s',
                              self.args.tf_timeout, e)
            return None

    def cb(self, msg):
        tf_msg = self.lookup(msg.header.stamp)
        if tf_msg is None:
            return

        q = tf_msg.transform.rotation
        t = tf_msg.transform.translation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        origin = np.array([t.x, t.y, t.z])

        stamp = msg.header.stamp.to_sec()
        if self.first_stamp is None:
            self.first_stamp = stamp
        self.poses.append((stamp - self.first_stamp, t.x, t.y, t.z))
        self.n_frames += 1

        arr = np.frombuffer(msg.data, dtype=struct_dtype(msg),
                            count=msg.height * msg.width)
        self.n_in += len(arr)

        if self.args.dry_run:
            if self.n_frames % 10 == 1:
                rospy.loginfo('t=%6.2f s  pose (%+7.3f %+7.3f %+7.3f)  %d pts',
                              stamp - self.first_stamp, t.x, t.y, t.z, len(arr))
            return

        pts = np.column_stack([arr['x'].astype(np.float64),
                               arr['y'].astype(np.float64),
                               arr['z'].astype(np.float64)])
        in_map = pts.dot(R.T) + origin          # rotate, then translate

        # de-rotate into arena coordinates before the box test, so the box
        # can be given in the room's own axes rather than the map's
        if self.args.yaw:
            a = np.radians(self.args.yaw)
            c, s = np.cos(a), np.sin(a)
            u = in_map[:, 0] * c + in_map[:, 1] * s
            v = -in_map[:, 0] * s + in_map[:, 1] * c
            in_map = np.column_stack([u, v, in_map[:, 2]])

        b = self.args.box
        keep = ((in_map[:, 0] >= b[0]) & (in_map[:, 0] <= b[1]) &
                (in_map[:, 1] >= b[2]) & (in_map[:, 1] <= b[3]) &
                (in_map[:, 2] >= b[4]) & (in_map[:, 2] <= b[5]))

        out = arr[keep]                          # ORIGINAL coordinates
        self.n_kept += len(out)

        msg_out = PointCloud2()
        msg_out.header = msg.header
        msg_out.height = 1
        msg_out.width = len(out)
        msg_out.fields = msg.fields
        msg_out.is_bigendian = msg.is_bigendian
        msg_out.point_step = msg.point_step
        msg_out.row_step = msg.point_step * len(out)
        msg_out.data = out.tobytes()
        msg_out.is_dense = True
        self.pub.publish(msg_out)

        if self.n_frames % 20 == 1:
            rospy.loginfo('t=%6.2f s  kept %d of %d (%.1f%%)',
                          stamp - self.first_stamp, len(out), len(arr),
                          100.0 * len(out) / max(1, len(arr)))

    def report(self):
        if not self.poses:
            rospy.logwarn('no frames processed; %d tf lookups failed',
                          self.n_failed)
            return

        p = np.array(self.poses)
        print('')
        print('=' * 62)
        print('frames processed : %d  (%d tf lookups failed)'
              % (self.n_frames, self.n_failed))
        print('duration         : %.2f s' % p[-1, 0])
        print('start pose       : %+7.3f %+7.3f %+7.3f' % tuple(p[0, 1:]))
        print('end pose         : %+7.3f %+7.3f %+7.3f' % tuple(p[-1, 1:]))
        print('net displacement : %.3f m'
              % np.linalg.norm(p[-1, 1:] - p[0, 1:]))
        path = np.linalg.norm(np.diff(p[:, 1:], axis=0), axis=1).sum()
        print('path length      : %.3f m' % path)

        if self.args.stationary:
            lo, hi = self.args.stationary
            w = p[(p[:, 0] >= lo) & (p[:, 0] <= hi)]
            print('')
            print('stationary window %.1f-%.1f s : %d frames' % (lo, hi, len(w)))
            if len(w) > 1:
                span = np.linalg.norm(w[:, 1:] - w[0, 1:], axis=1).max()
                print('  max excursion from first pose : %.4f m' % span)
                print('  >> should be near zero. Metres here means the tf')
                print('     chain or the frame names are wrong.')

        if not self.args.dry_run:
            print('')
            print('points kept : %d of %d (%.1f%%)'
                  % (self.n_kept, self.n_in,
                     100.0 * self.n_kept / max(1, self.n_in)))
            print('avg per scan: %d' % (self.n_kept // max(1, self.n_frames)))
        print('=' * 62)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/velodyne_points')
    ap.add_argument('--out-topic', default='/velodyne_points_cropped')
    ap.add_argument('--target-frame', default='map')
    ap.add_argument('--source-frame', default='base_link')
    ap.add_argument('--box', type=float, nargs=6, default=None,
                    metavar=('XMIN', 'XMAX', 'YMIN', 'YMAX', 'ZMIN', 'ZMAX'))
    ap.add_argument('--yaw', type=float, default=0.0,
                    help='arena yaw in the map frame, degrees '
                         '(from measure_arena.py). Box bounds are then in '
                         'arena u,v coordinates.')
    ap.add_argument('--tf-timeout', type=float, default=2.0,
                    help='seconds to wait for the transform at a scan stamp. '
                         'A timed-out scan is DROPPED from the output, and a run '
                         'of drops becomes a gap the odometry cannot track '
                         'across. The old 0.2 s default lost 13 of 998 scans. '
                         'Do not lower it without checking the failure count.')
    ap.add_argument('--dry-run', action='store_true')
    ap.add_argument('--stationary', default=None,
                    help='start,end in seconds, e.g. 0,6.5')
    args = ap.parse_args(rospy.myargv()[1:])

    if args.stationary:
        args.stationary = [float(v) for v in args.stationary.split(',')]
    if not args.dry_run and args.box is None:
        sys.exit('need --box, or use --dry-run')

    rospy.init_node('crop_map_frame', anonymous=True)
    Cropper(args)
    rospy.loginfo('cropping %s -> %s, %s frame%s',
                  args.in_topic, args.out_topic, args.target_frame,
                  (', arena yaw %.2f deg' % args.yaw) if args.yaw else '')
    rospy.spin()


if __name__ == '__main__':
    main()
