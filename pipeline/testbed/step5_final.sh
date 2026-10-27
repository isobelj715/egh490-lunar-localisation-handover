#!/bin/bash
# step5_final.sh -- regenerate every condition at a replay rate the pipeline can
# keep up with, then build the final table.
#
# WHAT IT RUNS
#   box-crop levels   L0 L1 L2 L3          played straight into LeGO-LOAM
#   filter conditions FIX100 FIX000        via filter_walls.py
#
# Usage:
#     ./step5_final.sh                      # 0.3x, ~45 min
#     RATE=0.5 ./step5_final.sh             # faster, verify the floor first
#     ONLY="FIX000 FIX100" ./step5_final.sh
#
# Run from the directory holding the bags, with roscore in another terminal.

set -u

WS="${WS:-$HOME/catkin_ws}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
FILT="${FILT:-$WS/pipeline/testbed/filter_walls.py}"
DERIVE="${DERIVE:-$HOME/catkin_ws/pipeline/testbed/derive_structure.py}"
TABLE="${TABLE:-./drift_table.py}"

RATE="${RATE:-0.3}"
OUT="${OUT:-results/testbed/final}"
SETTLE="${SETTLE:-8}"
SEED="${SEED:-1}"
L0BAG="${L0BAG:-2026-09-16-07-03-07.bag}"

MIN_INLIERS="${MIN_INLIERS:-400}"
MAX_PLANES="${MAX_PLANES:-12}"
MIN_RANGE="${MIN_RANGE:-2.0}"
MIN_EXTENT="${MIN_EXTENT:-1.0}"
MIN_Z_SPAN="${MIN_Z_SPAN:-0.20}"
MIN_COL_SPAN="${MIN_COL_SPAN:-0.10}"

CONDS=(
    "L0:$L0BAG:plain:no"
    "L1:level1.bag:plain:yes"
    "L2:level2.bag:plain:yes"
    "L3:level3.bag:plain:yes"
    "FIX100:fix100.bag:plain:yes"
    "FIX000:fix000.bag:plain:yes"
)

ONLY="${ONLY:-}"

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

[ -f "$REC" ] || { echo "missing record_run.py at: $REC" >&2; exit 1; }
[ -f "$FILT" ] || { echo "missing filter_walls.py at: $FILT" >&2; exit 1; }
grep -q 'def column_span' "$FILT" || {
    echo "ERROR: $FILT is the OLD copy (no column_span)." >&2; exit 1; }

rostopic list >/dev/null 2>&1 || {
    echo "ERROR: no ROS master. Start 'roscore' in another terminal first." >&2
    exit 1; }

python3 - level1.bag level2.bag level3.bag <<'PY' || exit 1
import os, struct, sys
import numpy as np
import rosbag

def stamps(p):
    b = rosbag.Bag(p)
    i = b.get_type_and_topic_info()[1]
    t = next((k for k, v in i.items()
              if v.msg_type == "sensor_msgs/PointCloud2"), None)
    o = []
    if t:
        for _, (dt, data, md5, pos, pt), _x in b.read_messages(topics=[t],
                                                              raw=True):
            _, sec, nsec = struct.unpack_from("<III", data, 0)
            o.append(int(round((sec + nsec * 1e-9) * 1000)))
    b.close()
    return np.array(sorted(o), dtype=np.int64)

have = {p: stamps(p) for p in sys.argv[1:] if os.path.exists(p)}
if not have:
    sys.exit(0)                      # nothing to check; conditions may be fine

bad = False
for p, st in have.items():
    if len(st) < 2:
        print("ERROR: %s has %d clouds" % (p, len(st))); bad = True; continue
    d = np.diff(st); med = int(np.median(d))
    k = int(np.argmax(d))
    # 3x the nominal interval = two or more consecutive scans absent. One
    # missing scan is a 0.2 s hole the odometry rides through; nine of them was
    # the 1.0 s hole that produced 5.5 m of false drift.
    if d.max() > 3 * med:
        print("ERROR: %s has a %.3f s gap at t=%.1f s (nominal %.3f s)"
              % (p, d.max()/1000.0, (st[k]-st[0])/1000.0, med/1000.0))
        bad = True
    elif d.max() > 1.5 * med:
        print("note: %s has one scan missing at t=%.1f s -- harmless, and the"
              % (p, (st[k]-st[0])/1000.0))
        print("      identical-stamps check below confirms it affects all levels.")

names = list(have)
if len(names) > 1:
    ref = have[names[0]]
    if not all(np.array_equal(have[n], ref) for n in names[1:]):
        print("ERROR: the levels do not hold the same scans:")
        for n in names:
            print("   %-14s %d clouds" % (n, len(have[n])))
        bad = True

if bad:
    print("Run ./verify_levels.py for detail, then ./step0_recrop.sh to rebuild.")
    sys.exit(1)
PY

echo "=============================================================="
echo "step5: final regeneration at replay rate $RATE"
echo "  output $OUT"
echo "  a 99 s bag takes $(python3 -c "print('%.1f' % (99/$RATE/60))") min per condition"
echo "=============================================================="

START=$(date +%s)

for entry in "${CONDS[@]}"; do
    IFS=':' read -r NAME BAG KIND REMAP <<< "$entry"

    if [ -n "$ONLY" ] && ! echo " $ONLY " | grep -q " $NAME "; then
        continue
    fi
    if [ ! -f "$BAG" ]; then
        case "$NAME" in
          FIX*) echo "missing $BAG -- build it first (no roscore needed):"
                KFV=$([ "$NAME" = FIX100 ] && echo 1.0 || echo 0.0)
                echo "   python3 make_filtered_bag.py level1.bag $BAG --keep-fraction $KFV"
                echo "   skipping $NAME";;
          *)    echo "missing bag: $BAG -- skipping $NAME";;
        esac
        continue
    fi
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        echo "== $NAME already done, skipping =="; continue
    fi

    echo
    echo "=============================================================="
    echo "== $NAME   $BAG   $([ "$KIND" = plain ] \
         && echo "direct replay" || echo "filter, keep_fraction=$KIND")"
    echo "=============================================================="

    # Cold start. timeLastProcessing, the gtsam graph and cloudKeyPoses3D all
    # persist when a bag ends; a warm mapOptmization produces no key poses.
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

    FILT_PID=""
    if [ "$KIND" != "plain" ]; then
        python3 "$FILT" '~input:=/vp_raw' '~output:=/velodyne_points' \
            _keep_fraction:="$KIND" _keep_sector_deg:=360 _seed:="$SEED" \
            _min_inliers:="$MIN_INLIERS" _max_planes:="$MAX_PLANES" \
            _min_range:="$MIN_RANGE" _min_extent:="$MIN_EXTENT" \
            _min_z_span:="$MIN_Z_SPAN" _min_col_span:="$MIN_COL_SPAN" \
            _debug:=false _publish_debug:=false \
            > "$OUT/${NAME}_filter.log" 2>&1 &
        FILT_PID=$!
        sleep 3
        kill -0 $FILT_PID 2>/dev/null || {
            echo "filter_walls died on startup:"; cat "$OUT/${NAME}_filter.log"
            kill -INT $LAUNCH_PID 2>/dev/null; exit 1; }
    fi

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init --note "rate=$RATE kind=$KIND" \
        > "$OUT/${NAME}_rec.log" 2>&1 &
    REC_PID=$!
    sleep 2

    echo "-- playing $BAG at rate $RATE"
    if [ "$KIND" != "plain" ]; then
        # bag -> filter -> LeGO-LOAM
        rosbag play --clock --rate "$RATE" "$BAG" \
            /velodyne_points_cropped:=/vp_raw
    elif [ "$REMAP" = "yes" ]; then
        rosbag play --clock --rate "$RATE" "$BAG" \
            /velodyne_points_cropped:=/velodyne_points
    else
        rosbag play --clock --rate "$RATE" "$BAG"
    fi

    sleep "$SETTLE"
    kill -INT $REC_PID  2>/dev/null; wait $REC_PID  2>/dev/null
    [ -n "$FILT_PID" ] && { kill -INT $FILT_PID 2>/dev/null
                            wait $FILT_PID 2>/dev/null; }
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    sleep 3

    [ -n "$FILT_PID" ] && grep "filter_walls summary" \
        "$OUT/${NAME}_filter.log" | tail -1 | sed 's/^/   /'
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        python3 - "$OUT/est_raw_$NAME.txt" <<'PY'
import numpy as np, sys
a = np.loadtxt(sys.argv[1], ndmin=2)
d = a[-1,0]-a[0,0]
print("   %d poses over %.1f s (%.2f Hz)%s"
      % (len(a), d, len(a)/max(d,1e-6),
         "   <-- TRUNCATED" if d < 95 else ""))
PY
    fi
done

echo
echo "elapsed $(( ($(date +%s) - START) / 60 )) min"

echo
echo "=============================================================="
echo "structure metrics"
echo "=============================================================="
for d in "$OUT"/logs/*/; do
    [ -d "$d" ] || continue
    [ -f "$d/structure.csv" ] && continue
    python3 "$DERIVE" "$d" >/dev/null 2>&1 || echo "   derive failed for $d"
done

python3 - "$OUT" "$RATE" <<'PY'
import glob, os, sys
import numpy as np
import pandas as pd

out, rate = sys.argv[1], sys.argv[2]
order = ["L0", "L1", "FIX100", "FIX000", "L2", "L3"]

lam, deg, nsc = {}, {}, {}
for p in glob.glob(os.path.join(out, "logs", "*", "structure.csv")):
    n = os.path.basename(os.path.dirname(p))
    df = pd.read_csv(p)
    if "map_full_lambda_min" not in df.columns or df.empty:
        continue
    lam[n] = float(df.map_full_lambda_min.median())
    nsc[n] = len(df)
    d = df.get("is_degenerate")
    deg[n] = float(d.mean()) * 100 if d is not None else float("nan")

print("\n%-8s %6s %11s %9s" % ("cond", "scans", "lambda_min", "degen %"))
print("-" * 38)
for n in order:
    if n in lam:
        print("%-8s %6d %11.1f %9.1f" % (n, nsc[n], lam[n], deg[n]))

print("\nrate %s. lambda_min run-to-run sd is ~1.5%%, so differences below ~3%%"
      % rate)
print("are not meaningful.")

if "FIX100" in lam and "L1" in lam:
    e = abs(lam["FIX100"] - lam["L1"]) / lam["L1"] * 100
    print("\nCONTROL  FIX100 %.1f vs L1 %.1f -- %.1f%% %s"
          % (lam["FIX100"], lam["L1"], e, "OK" if e < 10 else "FAILED"))
    if e >= 10:
        print("  The passthrough should reproduce L1. Stop here and find out why")
        print("  before reading anything else in this table.")

if "FIX100" in lam and "FIX000" in lam:
    print("EFFECT   resolvable walls removed: %.1f -> %.1f (%.1f%% drop)"
          % (lam["FIX100"], lam["FIX000"],
             (lam["FIX100"] - lam["FIX000"]) / lam["FIX100"] * 100))

files = {n: os.path.join(out, "est_raw_%s.txt" % n)
         for n in order if os.path.exists(os.path.join(out, "est_raw_%s.txt" % n))}
if "L0" in files and len(files) > 1:
    print("\nnow the drift table, on a window every run covers:\n")
    cmd = ("python3 ./drift_table.py --baseline %s %s"
           % (files["L0"], " ".join(v for k, v in files.items() if k != "L0")))
    print("  " + cmd)
    print("""
Read it as: median for typical tracking error, final for where it ended up, and
pathlen against L0's -- a longer path over the same real motion is wandering.
The noise floor at 0.3x is 0.012 m median / 0.023 m final, so anything above
~0.05 m is the manipulation rather than the pipeline.""")
