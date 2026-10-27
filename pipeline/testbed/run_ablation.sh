#!/bin/bash
# run_ablation.sh -- the four-level ablation for ONE recording.
#
# Plays the source bag and its three cropped levels through LeGO-LOAM, records a
# trajectory and the structure metrics for each
#
# Every level is the same physical drive with points deleted offline, so the
# rover's real motion is identical across all four. That is what makes the
# comparison controlled even though the runs are teleoperated and cannot be
# repeated.
#
# REPLAY RATE
# 0.3x. At full rate the pipeline drops scans and the pose stream is not
# reproducible -- two identical runs differed by metres, against 0.012 m at
# 0.3x, and every drift number measured that way sat inside the noise. Each
# condition also gets a COLD START: timeLastProcessing, the gtsam graph and
# cloudKeyPoses3D all persist when a bag ends, and a warm mapOptmization
# silently produces no key poses at all.
#
# STOCK SETTINGS
# This runs LeGO-LOAM unmodified, which is what a user of it experiences. The
# degeneracy brake stays on. If you want the brake-off comparison, that is a
# separate labelled experiment with its own build -- do not mix the two.
#
# Resumable: a level whose est_raw_*.txt already exists is skipped.
#
# Usage:
#   RUN=rectangle BAG=rectangle_2026-10-07-06-53-20.bag ./run_ablation.sh
#   RUN=rectangle BAG=... ONLY="L0 L2" ./run_ablation.sh
#
# Needs roscore in another terminal.

set -u

RUN="${RUN:?set RUN, e.g. RUN=rectangle}"
BAG="${BAG:?set BAG, the uncropped source recording}"
WS="${WS:-$HOME/catkin_ws}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
DERIVE="${DERIVE:-$HOME/catkin_ws/pipeline/testbed/derive_structure.py}"
RATE="${RATE:-0.3}"
OUT="${OUT:-results/testbed/$RUN}"
SETTLE="${SETTLE:-8}"
ONLY="${ONLY:-}"

# name : bag : needs the cropped-topic remap
CONDS=(
    "L0:$BAG:no"
    "L1:${RUN}_level1.bag:yes"
    "L2:${RUN}_level2.bag:yes"
    "L3:${RUN}_level3.bag:yes"
)

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

[ -f "$REC" ] || { echo "missing record_run.py at $REC" >&2; exit 1; }
rostopic list >/dev/null 2>&1 || {
    echo "ERROR: no ROS master. Start 'roscore' in another terminal." >&2
    exit 1; }

in_scope() {  # is this condition going to run?
    [ -z "$ONLY" ] && return 0
    echo " $ONLY " | grep -q " $1 "
}

# The levels must have no INTERIOR gaps and must hold the same scans as each
# other. Scans absent from the start or end of every level are fine -- the crop
# node cannot transform a scan before tf has published. An interior gap is not:
# it leaves the odometry nothing to track across, and the divergence looks
# exactly like a structure effect.
#
# Skipped when ONLY asks for L0 alone, so a baseline-only rerun does not need
# the level bags to exist.
CHECK=""
for n in L1 L2 L3; do in_scope "$n" && CHECK="$CHECK ${RUN}_level${n#L}.bag"; done
[ -n "$CHECK" ] && { python3 - $CHECK <<'PY' || exit 1
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
missing = [p for p in sys.argv[1:] if p not in have]
if missing:
    print("ERROR: missing level bags: %s" % ", ".join(missing))
    print("Build them with crop_run.sh first.")
    sys.exit(1)

bad = False
for p, st in have.items():
    if len(st) < 2:
        print("ERROR: %s has %d clouds" % (p, len(st))); bad = True; continue
    d = np.diff(st); med = int(np.median(d)); k = int(np.argmax(d))
    if d.max() > 3 * med:
        print("ERROR: %s has a %.3f s gap at t=%.1f s (nominal %.3f s)"
              % (p, d.max()/1000.0, (st[k]-st[0])/1000.0, med/1000.0))
        bad = True

names = list(have)
ref = have[names[0]]
if not all(np.array_equal(have[n], ref) for n in names[1:]):
    print("ERROR: the levels do not hold the same scans:")
    for n in names:
        print("   %-28s %d clouds" % (n, len(have[n])))
    bad = True

if bad:
    sys.exit(1)
print("levels verified: %d identical scans, no interior gaps" % len(ref))
PY
}

echo "=============================================================="
echo "ablation: $RUN"
echo "  source $BAG"
echo "  rate   $RATE   out $OUT"
echo "=============================================================="
START=$(date +%s)

for entry in "${CONDS[@]}"; do
    IFS=':' read -r NAME CBAG REMAP <<< "$entry"
    in_scope "$NAME" || continue
    [ -f "$CBAG" ] || { echo "missing $CBAG -- skipping $NAME"; continue; }
    [ -f "$OUT/est_raw_$NAME.txt" ] && { echo "== $NAME done, skipping =="
                                         continue; }

    echo
    echo "=============================================================="
    echo "== $NAME   $CBAG"
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
    for _ in $(seq 60); do
        rostopic list 2>/dev/null | grep -q /aft_mapped_to_init && break
        sleep 1
    done
    sleep 2

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init --note "$RUN $NAME rate=$RATE" \
        > "$OUT/${NAME}_rec.log" 2>&1 &
    REC_PID=$!
    sleep 2

    if [ "$REMAP" = "yes" ]; then
        rosbag play --clock --rate "$RATE" "$CBAG" \
            /velodyne_points_cropped:=/velodyne_points
    else
        rosbag play --clock --rate "$RATE" "$CBAG"
    fi

    sleep "$SETTLE"
    kill -INT $REC_PID 2>/dev/null; wait $REC_PID 2>/dev/null
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    sleep 3

    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        python3 - "$OUT/est_raw_$NAME.txt" <<'PY'
import numpy as np, sys
a = np.loadtxt(sys.argv[1], ndmin=2)
d = a[-1,0]-a[0,0]
print("   %d poses over %.1f s (%.2f Hz)%s"
      % (len(a), d, len(a)/max(d,1e-6), "   <-- TRUNCATED" if d < 90 else ""))
PY
    else
        echo "   NO TRAJECTORY -- check $OUT/${NAME}_lego.log"
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

python3 - "$OUT" "$RUN" <<'PY'
import glob, os, sys
import numpy as np
try:
    import pandas as pd
except ImportError:
    sys.exit(0)

out, run = sys.argv[1], sys.argv[2]
order = ["L0", "L1", "L2", "L3"]
rows = {}
for p in glob.glob(os.path.join(out, "logs", "*", "structure.csv")):
    n = os.path.basename(os.path.dirname(p))
    d = pd.read_csv(p)
    if "map_full_lambda_min" not in d.columns or d.empty:
        continue
    g = d.get("is_degenerate")
    rows[n] = (len(d), float(d.map_full_lambda_min.median()),
               float(d.map_rot_lambda_min.median())
               if "map_rot_lambda_min" in d else float("nan"),
               float(d.map_trans_lambda_min.median())
               if "map_trans_lambda_min" in d else float("nan"),
               float(g.mean()) * 100 if g is not None else float("nan"))
