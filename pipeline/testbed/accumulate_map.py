#!/usr/bin/env python
"""
Verify point-cloud transformations independently of
trajectory comparison by accumulating scans in the map frame.

Compare transform conventions using voxel consistency
and visual alignment in RViz.

Fewer voxels and higher points-per-voxel is the correct transform. If
--transpose wins, the transform line is backwards.

Also publishes the running cloud on /accumulated_map. View in RViz with
Fixed Frame = map: walls should be thin straight lines forming a rectangle,
not arcs or a double image.

Run while replaying the scan bag together with the tf bag, LeGO-LOAM off:

    roscore
    rosparam set use_sim_time true
    python3 accumulate_map.py
    rosbag play --clock gz_loop.bag gz_tf.bag
"""

import argparse

import numpy as np
import rospy
import tf2_ros
from sensor_msgs.msg import PointCloud2, PointField
from std_msgs.msg import Header
from tf.transformations import quaternion_matrix, euler_from_quaternion

DTYPES = {
    1: np.int8,   2: np.uint8,  3: np.int16,   4: np.uint16,
    5: np.int32,  6: np.uint32, 7: np.float32, 8: np.float64,
}


def struct_dtype(msg):
    return np.dtype({
        'names': [f.name for f in msg.fields],
        'formats': [DTYPES[f.datatype] for f in msg.fields],
        'offsets': [f.offset for f in msg.fields],
        'itemsize': msg.point_step,
    })


def xyz_cloud(points, frame):
    msg = PointCloud2()
    msg.header = Header(stamp=rospy.Time.now(), frame_id=frame)
    msg.height = 1
    msg.width = len(points)
    msg.fields = [
        PointField('x', 0, PointField.FLOAT32, 1),
        PointField('y', 4, PointField.FLOAT32, 1),
        PointField('z', 8, PointField.FLOAT32, 1),
    ]
    msg.is_bigendian = False
    msg.point_step = 12
    msg.row_step = 12 * len(points)
    msg.data = points.astype(np.float32).tobytes()
    msg.is_dense = True
    return msg


class Accumulator(object):
    def __init__(self, args):
        self.args = args
        self.chunks = []
        self.n = 0
        self.frames = 0
        self.yaws = []

        self.buf = tf2_ros.Buffer(rospy.Duration(3600.0))
        self.listener = tf2_ros.TransformListener(self.buf)
        self.pub = rospy.Publisher('/accumulated_map', PointCloud2, queue_size=2)
        rospy.Subscriber(args.in_topic, PointCloud2, self.cb, queue_size=5)
        rospy.on_shutdown(self.report)

    def cb(self, msg):
        self.frames += 1
        if self.frames % self.args.every:
            return
        try:
            tr = self.buf.lookup_transform(self.args.target_frame,
                                           self.args.source_frame,
                                           msg.header.stamp,
                                           rospy.Duration(0.2))
        except Exception:
            return

        q = tr.transform.rotation
        t = tr.transform.translation
        R = quaternion_matrix([q.x, q.y, q.z, q.w])[:3, :3]
        origin = np.array([t.x, t.y, t.z])
        self.yaws.append(np.degrees(
            euler_from_quaternion([q.x, q.y, q.z, q.w])[2]))

        arr = np.frombuffer(msg.data, dtype=struct_dtype(msg),
                            count=msg.height * msg.width)
        pts = np.column_stack([arr['x'].astype(np.float64),
                               arr['y'].astype(np.float64),
                               arr['z'].astype(np.float64)])
        pts = pts[np.isfinite(pts).all(axis=1)]
        if self.args.subsample > 1:
            pts = pts[::self.args.subsample]
        if not len(pts):
            return

        # the line under test
        if self.args.transpose:
            in_map = pts.dot(R) + origin          # deliberately backwards
        else:
            in_map = pts.dot(R.T) + origin

        self.chunks.append(in_map)
        self.n += len(in_map)
        while self.n > self.args.max_points and len(self.chunks) > 1:
            self.n -= len(self.chunks.pop(0))

        self.pub.publish(xyz_cloud(np.vstack(self.chunks),
                                   self.args.target_frame))

    def report(self):
        if not self.chunks:
            rospy.logwarn('nothing accumulated')
            return
        allp = np.vstack(self.chunks)
        y = np.array(self.yaws)
        vs = self.args.voxel
        vox = np.unique(np.floor(allp / vs).astype(np.int64), axis=0)
        ppv = len(allp) / float(len(vox))

        print('')
        print('=' * 62)
        print('MODE              : %s'
              % ('TRANSPOSED (control)' if self.args.transpose else 'normal'))
        print('scans accumulated : %d' % len(self.chunks))
        print('points            : %d' % len(allp))
        print('yaw range covered : %.1f deg' % (y.max() - y.min()))
        print('')
        print('extent in map frame:')
        for i, ax in enumerate('xyz'):
            print('  %s : %+7.2f .. %+7.2f  (span %6.2f m)'
                  % (ax, allp[:, i].min(), allp[:, i].max(),
                     allp[:, i].max() - allp[:, i].min()))
        print('')
        print('OVERLAP  (the decisive measure)')
        print('  voxel size            : %.2f m' % vs)
        print('  occupied voxels       : %d' % len(vox))
        print('  points per voxel      : %.3f' % ppv)
        print('')
        print('Run again with the other mode and compare. FEWER voxels and')
        print('HIGHER points-per-voxel is the correct transform: scans that')
        print('land on top of each other overlap, smeared ones do not.')
        print('In RViz (Fixed Frame = map, /accumulated_map) the correct one')
        print('shows thin straight walls, not arcs or a double image.')

        if self.args.save_pcd:
            self.save_pcd(allp)
        print('=' * 62)

    def save_pcd(self, pts):
        """Write a binary PCD. No voxel downsampling: the point of saving
        this rather than /laser_cloud_surround is to keep the resolution
        needed to measure a wall a few tenths of a metre tall."""
        path = self.args.save_pcd
        xyz = pts.astype(np.float32)
        header = (
            '# .PCD v0.7 - accumulated by accumulate_map.py\n'
            'VERSION 0.7\n'
            'FIELDS x y z\n'
            'SIZE 4 4 4\n'
            'TYPE F F F\n'
            'COUNT 1 1 1\n'
            'WIDTH %d\n'
            'HEIGHT 1\n'
            'VIEWPOINT 0 0 0 1 0 0 0\n'
            'POINTS %d\n'
            'DATA binary\n' % (len(xyz), len(xyz))
        )
        with open(path, 'wb') as fh:
            fh.write(header.encode('ascii'))
            fh.write(np.ascontiguousarray(xyz).tobytes())
        print('')
        print('wrote %s  (%d points, %.1f MB)'
              % (path, len(xyz), len(xyz) * 12 / 1e6))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--in-topic', default='/velodyne_points')
    ap.add_argument('--target-frame', default='map')
    ap.add_argument('--source-frame', default='base_link')
    ap.add_argument('--every', type=int, default=2,
                    help='use every Nth scan (default 2)')
    ap.add_argument('--subsample', type=int, default=3,
                    help='keep every Nth point within a scan (default 3)')
    ap.add_argument('--max-points', type=int, default=600000,
                    help='ring-buffer cap; raise it when saving a full map')
    ap.add_argument('--voxel', type=float, default=0.10,
                    help='voxel size for the overlap measure (default 0.10 m)')
    ap.add_argument('--transpose', action='store_true',
                    help='apply the rotation backwards, as a control')
    ap.add_argument('--save-pcd', default=None, metavar='PATH',
                    help='write the accumulated cloud to a binary PCD on exit')
    args = ap.parse_args(rospy.myargv()[1:])

    rospy.init_node('accumulate_map', anonymous=True)
    Accumulator(args)
    rospy.loginfo('accumulating %s in %s frame%s -> /accumulated_map',
                  args.in_topic, args.target_frame,
                  ' [TRANSPOSED CONTROL]' if args.transpose else '')
    rospy.spin()


if __name__ == '__main__':
    main()
