#!/usr/bin/env python3
"""
Record ground truth and LeGO-LOAM estimate for one named run.


Usage:
    python3 pipeline/testbed/record_run.py L0
    python3 record_run.py L0 --outdir results/testbed/<run>

Start this BEFORE drive_scripted_path.py. Ctrl-C to stop and save.
"""

import argparse
import json
import os
import subprocess
import sys

import numpy as np
import rospy
from nav_msgs.msg import Odometry

GT_TOPIC = "/ground_truth/state"

EST_TOPIC = "/aft_mapped_to_init"

def make_row(msg):
    p = msg.pose.pose.position
    q = msg.pose.pose.orientation
    # header.stamp, not Time.now(): both topics share the sim clock, and the
    # header stamp is when the pose was valid rather than when it arrived.
    return [msg.header.stamp.to_sec(), p.x, p.y, p.z, q.x, q.y, q.z, q.w]


def git_sha():
    try:
        return subprocess.check_output(
            ["git", "rev-parse", "HEAD"], stderr=subprocess.DEVNULL
        ).decode().strip()
    except Exception:
        return None


class Recorder:
    def __init__(self, args):
        self.args = args
        self.gt, self.est = [], []
        self.moved = False
        self.still_since = None
        self.gt_path = os.path.join(args.outdir, f"gt_raw_{args.run}.txt")
        self.est_path = os.path.join(args.outdir, f"est_raw_{args.run}.txt")
        self.meta_path = os.path.join(args.outdir, f"meta_{args.run}.json")

    def check_clear(self):
        existing = [p for p in (self.gt_path, self.est_path) if os.path.exists(p)]
        if existing and not self.args.force:
            sys.exit(f"refusing to overwrite: {', '.join(existing)}\n"
                     f"pick another run name, or pass --force")

    def save(self):
        os.makedirs(self.args.outdir, exist_ok=True)
        meta = {
            "run": self.args.run,
            "gt_topic": self.args.gt_topic,
            "est_topic": self.args.est_topic,
            "git_sha": git_sha(),
            "trajectory": self.args.trajectory or self.args.run.split("_")[0],
            "note": self.args.note,
        }

        for name, poses, path in (("gt", self.gt, self.gt_path),
                                  ("est", self.est, self.est_path)):
            if not poses:
                print(f"WARNING: no {name} poses recorded on "
                      f"{self.args.gt_topic if name == 'gt' else self.args.est_topic}")
                meta[f"{name}_poses"] = 0
                continue
            a = np.array(poses)
            np.savetxt(path, a, fmt="%.9f")
            dur = a[-1, 0] - a[0, 0]
            rate = (len(a) - 1) / dur if dur > 0 else float("nan")
            dist = float(np.sum(np.linalg.norm(np.diff(a[:, 1:4], axis=0), axis=1)))
            meta[f"{name}_poses"] = len(a)
            meta[f"{name}_duration_s"] = round(dur, 3)
            meta[f"{name}_rate_hz"] = round(rate, 2)
            meta[f"{name}_path_len_m"] = round(dist, 4)
            if name == "gt":
                meta["spawn_xyz"] = [round(v, 4) for v in a[0, 1:4]]
                meta["spawn_quat_xyzw"] = [round(v, 6) for v in a[0, 4:8]]
            print(f"Saved {len(a)} {name} poses -> {path}  "
                  f"({rate:.1f} Hz over {dur:.1f} s, {dist:.2f} m)")

        with open(self.meta_path, "w") as fh:
            json.dump(meta, fh, indent=2)
        print(f"Saved metadata -> {self.meta_path}")

        if meta.get("gt_poses") and meta.get("est_poses"):
            overlap = (min(self.gt[-1][0], self.est[-1][0])
                       - max(self.gt[0][0], self.est[0][0]))
            if overlap <= 0:
                print("WARNING: GT and estimate timestamps do not overlap at all")
            elif overlap < 0.8 * meta["gt_duration_s"]:
                print(f"WARNING: only {overlap:.1f} s of overlap against "
                      f"{meta['gt_duration_s']:.1f} s of GT -- did the estimate "
                      f"start late, or did LeGO-LOAM lose tracking?")

    def watchdog(self, _evt):
        if self.args.stop_after_still <= 0 or len(self.gt) < 3:
            return
        a = np.array(self.gt[-40:])
        dt = a[-1, 0] - a[0, 0]
        if dt <= 0:
            return
        span = float(np.max(np.linalg.norm(a[:, 1:4] - a[0, 1:4], axis=1)))
        if span / dt > self.args.still_speed:
            self.moved = True
            self.still_since = None
            return
        if not self.moved:
            return                      # never started; ignore the initial wait
        now = a[-1, 0]
        if self.still_since is None:
            self.still_since = now
        elif now - self.still_since >= self.args.stop_after_still:
            rospy.loginfo(f"GT stationary for "
                          f"{self.args.stop_after_still:.1f}s -- stopping")
            rospy.signal_shutdown("trajectory complete")

    def run(self):
        rospy.init_node("trajectory_saver", anonymous=True)
        rospy.Subscriber(self.args.gt_topic, Odometry,
                         lambda m: self.gt.append(make_row(m)))
        rospy.Subscriber(self.args.est_topic, Odometry,
                         lambda m: self.est.append(make_row(m)))
        rospy.on_shutdown(self.save)
        if self.args.stop_after_still > 0:
            rospy.Timer(rospy.Duration(0.25), self.watchdog)
        print(f"Recording run '{self.args.run}'")
        print(f"  GT  {self.args.gt_topic}")
        print(f"  EST {self.args.est_topic}")
        print(f"  ->  {self.args.outdir}/")
        if self.args.stop_after_still > 0:
            print(f"  auto-stop after {self.args.stop_after_still:.1f}s stationary")
        print("Ctrl-C to stop and save.")
        rospy.spin()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("run", help="run name, e.g. L0")
    ap.add_argument("--outdir", default="results/testbed/<run>")
    ap.add_argument("--gt-topic", default=GT_TOPIC)
    ap.add_argument("--est-topic", default=EST_TOPIC)
    ap.add_argument("--trajectory", help="trajectory key, if not the run name prefix")
    ap.add_argument("--note", default="", help="free text, e.g. 'near north wall'")
    ap.add_argument("--force", action="store_true", help="overwrite an existing run")
    ap.add_argument("--stop-after-still", type=float, default=0.0, metavar="SEC",
                    help="auto-stop once GT has been stationary this long after "
                         "having moved; 0 disables. Recommended: 2.0")
    ap.add_argument("--still-speed", type=float, default=0.02,
                    help="GT speed (m/s) below which the rover counts as still")
    args = ap.parse_args(rospy.myargv()[1:])

    os.makedirs(args.outdir, exist_ok=True)
    rec = Recorder(args)
    rec.check_clear()
    rec.run()


if __name__ == "__main__":
    main()
