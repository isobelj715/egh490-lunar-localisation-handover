#!/bin/bash
# step2_sweep_walls.sh -- fill the lambda_min gap using partial wall retention.
#
# Chain per level, all in one pass -- no intermediate bags:
#     level1.bag -> /vp_raw -> filter_walls.py -> /velodyne_points -> LeGO-LOAM
#
# Usage:
#     ./step2_sweep_walls.sh
#     FRACS="1.0 0.5 0.0" ./step2_sweep_walls.sh

set -u

WS="${WS:-$HOME/catkin_ws}"
BAG="${BAG:-level1.bag}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
FILT="${FILT:-$WS/pipeline/testbed/filter_walls.py}"
OUT="${OUT:-results/testbed/sweep}"
SEED="${SEED:-1}"
MIN_INLIERS="${MIN_INLIERS:-2000}"
FRACS="${FRACS:-1.00 0.60 0.35 0.20 0.10 0.00}"

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

[ -f "$BAG" ] || { echo "missing bag: $BAG" >&2; exit 1; }
[ -f "$REC" ] || { echo "missing record_run.py at: $REC" >&2; exit 1; }
[ -f "$FILT" ] || { echo "missing filter_walls.py at: $FILT" >&2;
                    echo "find it with: find $WS -name filter_walls.py" >&2; exit 1; }

for KF in $FRACS; do
    NAME="KF$(echo "$KF" | tr -d '.')"
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        echo "== $NAME already done, skipping =="; continue
    fi

    echo
    echo "=============================================================="
    echo "== $NAME   keep_fraction=$KF   seed=$SEED"
    echo "=============================================================="

    rosnode kill -a 2>/dev/null
    killall -9 mapOptmization featureAssociation imageProjection \
               transformFusion 2>/dev/null
    sleep 3

    export LEGO_LOG_DIR="$PWD/$OUT/logs/$NAME"
    mkdir -p "$LEGO_LOG_DIR"
    rosparam set use_sim_time true

    roslaunch lego_loam run.launch rviz:=false > "$OUT/${NAME}_lego.log" 2>&1 &
    LAUNCH_PID=$!
    echo "-- waiting for /aft_mapped_to_init"
    for _ in $(seq 60); do
        rostopic list 2>/dev/null | grep -q /aft_mapped_to_init && break
        sleep 1
    done
    sleep 2

    # filter sits between the bag and LeGO-LOAM
    python3 "$FILT" ~input:=/vp_raw ~output:=/velodyne_points \
        _keep_fraction:="$KF" _keep_sector_deg:=360 _seed:="$SEED" \
        _min_inliers:="$MIN_INLIERS" _max_planes:="${MAX_PLANES:-6}" \
        > "$OUT/${NAME}_filter.log" 2>&1 &
    FILT_PID=$!
    sleep 3
    kill -0 $FILT_PID 2>/dev/null || {
        echo "filter_walls died on startup:"; cat "$OUT/${NAME}_filter.log"
        kill -INT $LAUNCH_PID 2>/dev/null; exit 1; }

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init --note "keep_fraction=$KF" \
        > "$OUT/${NAME}_rec.log" 2>&1 &
    REC_PID=$!
    sleep 2

    echo "-- playing $BAG"
    rosbag play --clock "$BAG" /velodyne_points_cropped:=/vp_raw

    sleep 8
    kill -INT $REC_PID  2>/dev/null; wait $REC_PID  2>/dev/null
    kill -INT $FILT_PID 2>/dev/null; wait $FILT_PID 2>/dev/null
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    sleep 3

    echo "-- filter summary --"
    grep "filter_walls summary" "$OUT/${NAME}_filter.log" | tail -1 \
        || echo "   (no summary -- check the topic remap)"
done

echo
echo "=============================================================="
ls -lh "$OUT"/est_raw_*.txt 2>/dev/null
echo
echo "Next:"
echo "  python3 \$HOME/catkin_ws/eval/analysis/derive_structure.py --batch $OUT/logs"
echo "  python3 sweep_table.py $OUT"
echo "=============================================================="
