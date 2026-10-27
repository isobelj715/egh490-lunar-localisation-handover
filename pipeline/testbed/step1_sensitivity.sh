#!/bin/bash
# step1_sensitivity.sh -- does level 3's result depend on where the box edge sits?
#
# Level 3's box is 0.4 m inside the retaining wall. This re-crops at 1.0 m inside and re-measures lambda_min.
#
#
# Needs roscore running in another terminal. LeGO-LOAM must NOT be running when
# the crop pass starts -- its tf would fight the replay.
#
# Usage:  ./step1_sensitivity.sh

set -u

WS="${WS:-$HOME/catkin_ws}"
BAG="${BAG:-2026-09-16-07-03-07.bag}"
TF="${TF:-yandiwanba3_tf.bag}"
YAW="${YAW:-29.75}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
OUT="${OUT:-results/testbed/levels}"

# retaining wall lines: u -9.60 / +1.70, v -2.89 / +16.5
# level 3  = 0.4 m inside   -> -9.2  1.3  -2.4  16.1
# level 3b = 1.0 m inside   -> -8.6  0.7  -1.89 15.5
BOX="-8.0 0.1 -1.29 14.9 -5 8"
NAME=L3c

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

for f in "$BAG" "$TF" ./crop_map_frame.py "$REC"; do
    [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
python3 ./crop_map_frame.py --help 2>&1 | grep -q -- --yaw || {
    echo "crop_map_frame.py has no --yaw; you have the old copy" >&2; exit 1; }

echo "=============================================================="
echo "PASS 1 of 2 -- cropping at 1.0 m inside the retaining wall"
echo "=============================================================="
pgrep -f mapOptmization >/dev/null && {
    echo "LeGO-LOAM is running -- stop it first" >&2; exit 1; }

rm -f level3b.bag
rosparam set use_sim_time true

python3 ./crop_map_frame.py --yaw "$YAW" --box $BOX > L3b_crop.log 2>&1 &
CROP_PID=$!
sleep 3
kill -0 $CROP_PID 2>/dev/null || { echo "crop node died:"; cat L3b_crop.log; exit 1; }

rosbag record /velodyne_points_cropped -O level3b.bag __name:=rec_L3b >/dev/null 2>&1 &
sleep 2
rosbag play --clock "$BAG" "$TF"
sleep 2
rosnode kill /rec_L3b >/dev/null 2>&1
kill -INT $CROP_PID 2>/dev/null; wait $CROP_PID 2>/dev/null
echo "-- crop summary --"; tail -8 L3b_crop.log

echo
echo "=============================================================="
echo "PASS 2 of 2 -- running level3b through LeGO-LOAM"
echo "=============================================================="
rosnode kill -a 2>/dev/null
killall -9 mapOptmization featureAssociation imageProjection transformFusion 2>/dev/null
sleep 3

export LEGO_LOG_DIR="$PWD/$OUT/logs/$NAME"
mkdir -p "$LEGO_LOG_DIR"
rosparam set use_sim_time true
roslaunch lego_loam run.launch rviz:=false > "$OUT/${NAME}_lego.log" 2>&1 &
LAUNCH_PID=$!
for _ in $(seq 60); do
    rostopic list 2>/dev/null | grep -q /aft_mapped_to_init && break; sleep 1
done
sleep 2
python3 "$REC" "$NAME" --outdir "$OUT" --force --est-topic /aft_mapped_to_init \
    > "$OUT/${NAME}_rec.log" 2>&1 &
REC_PID=$!
sleep 2
rosbag play --clock level3b.bag /velodyne_points_cropped:=/velodyne_points
sleep 8
kill -INT $REC_PID 2>/dev/null; wait $REC_PID 2>/dev/null
kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null

echo
echo "=============================================================="
echo "Now derive and compare:"
echo "  python3 \$HOME/catkin_ws/eval/analysis/derive_structure.py $OUT/logs/$NAME"
echo "  python3 -c \"import pandas as pd;"
echo "    [print('%-4s %8.2f' % (d, pd.read_csv('$OUT/logs/'+d+'/structure.csv')"
echo "     .map_full_lambda_min.median())) for d in ['L3','L3b']]\""
echo "=============================================================="
