#!/bin/bash
# crop_all.sh -- produce the three cropped bags for the ablation levels.
#
# Replays the source bag together with the tf bag recorded during an earlier
# unfiltered LeGO-LOAM pass, crops each scan to a box fixed in arena
# coordinates, and records the result. LeGO-LOAM must NOT be running: the
# poses come from the tf bag, and a live instance would publish competing tf.
#
# Level 0 is the source bag unmodified, so only three crops are produced.
#
# Usage:
#     ./crop_all.sh
#     BAG=other.bag TF=other_tf.bag ./crop_all.sh

set -u

BAG="${BAG:-2026-09-16-07-03-07.bag}"
TF="${TF:-yandiwanba3_tf.bag}"
YAW="${YAW:-29.75}"
CROP="${CROP:-./crop_map_frame.py}"

for f in "$BAG" "$TF" "$CROP"; do
    [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done

# level name, then the six box bounds in arena u,v,z
LEVELS=(
    "level1  -11.6 3.6 -5.6 18.4 -5 8"
    "level2  -10.3 2.3 -4.2 17.1 -5 8"
    "level3   -9.2 1.3 -2.4 16.1 -5 8"
)

pgrep -f mapOptmization >/dev/null && {
    echo "LeGO-LOAM is running -- stop it first, its tf will fight the replay" >&2
    exit 1
}

rosparam set use_sim_time true

for entry in "${LEVELS[@]}"; do
    read -r NAME B1 B2 B3 B4 B5 B6 <<< "$entry"
    OUT="${NAME}.bag"

    if [ -f "$OUT" ]; then
        echo "== $NAME: $OUT exists, skipping =="
        continue
    fi

    echo
    echo "=============================================================="
    echo "== $NAME   box $B1 $B2 $B3 $B4 $B5 $B6   yaw $YAW"
    echo "=============================================================="

    python3 "$CROP" --yaw "$YAW" --box "$B1" "$B2" "$B3" "$B4" "$B5" "$B6" \
        > "${NAME}_crop.log" 2>&1 &
    CROP_PID=$!
    sleep 3

    rosbag record /velodyne_points_cropped -O "$OUT" \
        __name:="rec_$NAME" > /dev/null 2>&1 &
    REC_PID=$!
    sleep 2

    rosbag play --clock "$BAG" "$TF"

    sleep 2
    rosnode kill "/rec_$NAME" > /dev/null 2>&1
    wait $REC_PID 2>/dev/null

    kill -INT $CROP_PID 2>/dev/null
    wait $CROP_PID 2>/dev/null

    echo "-- $NAME summary --"
    tail -12 "${NAME}_crop.log"
done

echo
echo "=============================================================="
ls -lh level*.bag 2>/dev/null
echo
echo "Kept-point percentages must fall level1 > level2 > level3."
echo "If they do not, a box bound is wrong. Check the *_crop.log files."
echo "=============================================================="
