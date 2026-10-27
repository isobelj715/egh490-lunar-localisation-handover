#!/bin/bash
# step0_recrop.sh -- rebuild the three cropped bags without dropped scans.
#
# WHAT THIS DOES
# Re-crops all three levels at a slow replay rate with the longer timeout, then
# VERIFIES each output against the source: same message count, no gaps, zero tf
# failures. A bag that fails verification is reported and not used.
#
# Usage:
#     ./step0_recrop.sh                 # 0.3x, ~17 min
#     RATE=0.5 ./step0_recrop.sh
#     FORCE=1 ./step0_recrop.sh         # re-crop even bags that verify clean
#
# LeGO-LOAM must NOT be running -- poses come from the tf bag and a live
# instance would publish competing tf. Needs roscore in another terminal.

set -u

BAG="${BAG:-2026-09-16-07-03-07.bag}"
TF="${TF:-yandiwanba3_tf.bag}"
YAW="${YAW:-29.75}"
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="$(cd "$HERE/../.." && pwd)"
CROP="${CROP:-$HERE/crop_map_frame.py}"
CHECK="${CHECK:-$WS/tools/check_bag_gaps.py}"
RATE="${RATE:-0.3}"
TF_TIMEOUT="${TF_TIMEOUT:-2.0}"
FORCE="${FORCE:-}"

# level name, then the six box bounds in arena u,v,z -- unchanged from crop_all.sh
LEVELS=(
    "level1  -11.6 3.6 -5.6 18.4 -5 8"
    "level2  -10.3 2.3 -4.2 17.1 -5 8"
    "level3   -9.2 1.3 -2.4 16.1 -5 8"
)

for f in "$BAG" "$TF" "$CROP"; do
    [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
grep -q 'tf-timeout' "$CROP" || {
    echo "ERROR: $CROP is the old copy (no --tf-timeout)." >&2
    echo "Install the updated crop_map_frame.py first." >&2; exit 1; }

pgrep -f mapOptmization >/dev/null && {
    echo "LeGO-LOAM is running -- stop it first" >&2; exit 1; }

# ---- how many clouds are in the source? ------------------------------
SRC_N=$(python3 - "$BAG" <<'PY'
import sys, rosbag
b = rosbag.Bag(sys.argv[1])
info = b.get_type_and_topic_info()[1]
n = max((v.message_count for v in info.values()
         if v.msg_type == "sensor_msgs/PointCloud2"), default=0)
b.close(); print(n)
PY
)
echo "=============================================================="
echo "step0: re-cropping at rate $RATE, tf timeout ${TF_TIMEOUT}s"
echo "  source $BAG has $SRC_N clouds -- every output must match it"
echo "=============================================================="

rosparam set use_sim_time true

for entry in "${LEVELS[@]}"; do
    read -r NAME B1 B2 B3 B4 B5 B6 <<< "$entry"
    OUT="${NAME}.bag"

    # Keep a clean bag; set aside a short one.
    if [ -f "$OUT" ] && [ -z "$FORCE" ]; then
        N=$(python3 - "$OUT" <<'PY'
import sys, rosbag
b = rosbag.Bag(sys.argv[1])
info = b.get_type_and_topic_info()[1]
print(max((v.message_count for v in info.values()
           if v.msg_type == "sensor_msgs/PointCloud2"), default=0))
b.close()
PY
)
        if [ "$N" = "$SRC_N" ]; then
            echo "== $NAME: $N clouds, already complete, skipping =="
            continue
        fi
        echo "== $NAME: has $N of $SRC_N clouds -- re-cropping"
        mv -f "$OUT" "${NAME}_dropped.bag"
        echo "   old bag kept as ${NAME}_dropped.bag"
    fi

    echo
    echo "=============================================================="
    echo "== $NAME   box $B1 $B2 $B3 $B4 $B5 $B6   yaw $YAW"
    echo "=============================================================="

    python3 "$CROP" --yaw "$YAW" --tf-timeout "$TF_TIMEOUT" \
        --box "$B1" "$B2" "$B3" "$B4" "$B5" "$B6" \
        > "${NAME}_crop.log" 2>&1 &
    CROP_PID=$!
    sleep 3
    kill -0 $CROP_PID 2>/dev/null || {
        echo "crop node died:"; cat "${NAME}_crop.log"; exit 1; }

    rosbag record /velodyne_points_cropped -O "$OUT" \
        __name:="rec_$NAME" > /dev/null 2>&1 &
    REC_PID=$!
    sleep 2

    rosbag play --clock --rate "$RATE" "$BAG" "$TF"

    sleep 3
    rosnode kill "/rec_$NAME" > /dev/null 2>&1
    wait $REC_PID 2>/dev/null
    kill -INT $CROP_PID 2>/dev/null
    wait $CROP_PID 2>/dev/null
    sleep 2

    echo "-- crop report --"
    grep -E "frames processed|kept" "${NAME}_crop.log" | tail -4 | sed 's/^/   /'
done

# ---- verify ----------------------------------------------------------
echo
echo "=============================================================="
echo "VERIFICATION"
echo "=============================================================="
python3 - "$BAG" "$SRC_N" level1.bag level2.bag level3.bag <<'PY'
import os
import sys

import numpy as np
import rosbag

src, src_n = sys.argv[1], int(sys.argv[2])
ok_all = True

print("%-16s %8s %8s %10s %s" % ("bag", "clouds", "missing", "max gap", "verdict"))
print("-" * 62)
for path in sys.argv[3:]:
    if not os.path.exists(path):
        print("%-16s %8s %8s %10s %s" % (path, "-", "-", "-", "MISSING"))
        ok_all = False
        continue
    b = rosbag.Bag(path)
    info = b.get_type_and_topic_info()[1]
    topic = next((t for t, v in info.items()
                  if v.msg_type == "sensor_msgs/PointCloud2"), None)
    stamps = []
    if topic:
        import struct
        for _, (dt, data, md5, pos, pt), t in b.read_messages(topics=[topic],
                                                             raw=True):
            _, sec, nsec = struct.unpack_from("<III", data, 0)
            stamps.append(sec + nsec * 1e-9)
    b.close()
    n = len(stamps)
    gaps = np.diff(np.array(stamps)) if n > 1 else np.array([0.0])
    med = float(np.median(gaps))
    mx = float(gaps.max())
    missing = src_n - n
    bad = (missing != 0) or (mx > 3 * med)
    ok_all = ok_all and not bad
    print("%-16s %8d %8d %9.3fs %s"
          % (path, n, missing, mx, "FAIL" if bad else "ok"))

print("")
if ok_all:
    print("""All three complete and gap-free. The levels now differ ONLY by their
crop boxes, which is what the ablation assumes. Run step5_final.sh.""")
else:
    print("""At least one bag still has drops. Check the tf failure count in the
*_crop.log files. If failures are non-zero, raise TF_TIMEOUT or lower RATE and
re-run with FORCE=1. Do not run step5_final.sh on a FAIL -- a gap mimics the
effect being measured.""")
PY
