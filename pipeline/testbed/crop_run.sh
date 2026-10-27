#!/bin/bash
# crop_run.sh -- build the three filtered levels for ONE recording.
#
# Each run has its own arena frame: LeGO-LOAM puts the map origin at the rover's
# start pose, so the yaw and the wall lines differ between recordings even
# though the arena never moves. This takes one run's measured geometry and
# derives the three level boxes from it, so the offsets are identical across
# runs and the levels mean the same thing everywhere.
#
# THE LEVELS, defined relative to the measured walls rather than by absolute
# numbers:
#   level1  everything inside the outer walls, including them
#           = outer walls + 0.85 m margin
#   level2  outer walls and the walkway gone, retaining wall kept
#           = retaining wall + 0.65 m, which also sits clear inside the outer
#   level3  sand, rocks and crater only
#           = retaining wall - 0.4 m
#
# WHAT GOES IN
# From measure_arena.py on that run's accumulated map:
#   YAW          arena yaw in the map frame
#   RETAINING    u_min u_max v_min v_max of the retaining wall
# A far wall the rover never got close to may be missing from the measurement --
# the lower part is occluded and a 0.4 m wall past 10 m is a one-beam target.
# Infer it from the known 11.3 x 19.4 m retaining rectangle and pass it anyway;
# the outer-wall spans confirm the arithmetic transfers.
#
# Usage:
#   RUN=rectangle BAG=rectangle_2026-10-07-06-53-20.bag TF=rectangle_tf.bag \
#   YAW=33.00 RETAINING="-10.29 1.11 -12.13 7.27" ./crop_run.sh
#
#   # level 3 again, this time with the overhead returns cut too:
#   RUN=zigzag TAG=_lim LEVELS=3 ZHI=-0.45 BAG=... TF=... \
#       YAW=33.00 RETAINING="..." ./crop_run.sh
#   -> zigzag_lim_level3.bag, leaving zigzag_level3.bag untouched
#
# LeGO-LOAM must NOT be running -- the poses come from the tf bag and a live
# instance would publish competing tf. Needs roscore in another terminal.

set -u

RUN="${RUN:?set RUN, e.g. RUN=rectangle}"
BAG="${BAG:?set BAG, the source recording}"
TF="${TF:-${RUN}_tf.bag}"
YAW="${YAW:?set YAW from measure_arena.py}"
RETAINING="${RETAINING:?set RETAINING=\"u_min u_max v_min v_max\"}"

CROP="${CROP:-./crop_map_frame.py}"
RATE="${RATE:-0.3}"
TF_TIMEOUT="${TF_TIMEOUT:-2.0}"
ZLO="${ZLO:--5}"
ZHI="${ZHI:-8}"
TAG="${TAG:-}"               # appended to RUN in output names
LEVELS="${LEVELS:-1 2 3}"    # which levels to build
# Must EXCEED the widest retaining-to-outer gap, or level1 clips an outer wall
# on the side where the walkway is widest. Measured gaps on this arena run
# 1.00-1.60 m, so 2.00 clears all four 
MARGIN1="${MARGIN1:-2.00}"   # outside the retaining wall -> beyond the outer
MARGIN2="${MARGIN2:-0.65}"   # outside the retaining wall, inside the outer
MARGIN3="${MARGIN3:-0.40}"   # inside the retaining wall

read -r RU0 RU1 RV0 RV1 <<< "$RETAINING"

for f in "$BAG" "$TF" "$CROP"; do
    [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
grep -q 'tf-timeout' "$CROP" || {
    echo "ERROR: $CROP is the old copy (no --tf-timeout). The 0.2 s default" >&2
    echo "drops scans on timeout and leaves gaps that mimic structure loss." >&2
    exit 1; }
pgrep -f mapOptmization >/dev/null && {
    echo "LeGO-LOAM is running -- stop it first" >&2; exit 1; }
rostopic list >/dev/null 2>&1 || {
    echo "ERROR: no ROS master. Start 'roscore' in another terminal." >&2
    exit 1; }

box() {  # margin -> "u0 u1 v0 v1 z0 z1", positive margin grows the box
    python3 -c "
m=float('$1')
print('%.2f %.2f %.2f %.2f %s %s' % ($RU0-m, $RU1+m, $RV0-m, $RV1+m,
                                     '$ZLO', '$ZHI'))"
}

B1=$(box "$MARGIN1"); B2=$(box "$MARGIN2"); B3=$(box "-$MARGIN3")

if [ -n "${OUTER:-}" ]; then
    read -r OU0 OU1 OV0 OV1 <<< "$OUTER"
    python3 -c "
b1='$B1'.split(); b2='$B2'.split()
u0,u1,v0,v1 = map(float, b1[:4]); w0,w1,x0,x1 = map(float, b2[:4])
o = ($OU0, $OU1, $OV0, $OV1)
bad = []
if u0 > o[0] or u1 < o[1] or v0 > o[2] or v1 < o[3]:
    bad.append('level1 does NOT enclose the outer walls -- raise MARGIN1')
if w0 < o[0] or w1 > o[1] or x0 < o[2] or x1 > o[3]:
    bad.append('level2 reaches OUTSIDE the outer walls -- lower MARGIN2')
for b in bad: print('   !! ' + b)
if not bad: print('   level1 encloses the outer walls, level2 sits inside them')
" || true
fi

echo "=============================================================="
echo "cropping $RUN"
echo "  source      $BAG"
echo "  tf          $TF"
echo "  yaw         $YAW deg"
echo "  retaining   u $RU0 .. $RU1   v $RV0 .. $RV1"
echo "  level1 box  $B1   (retaining +${MARGIN1} m, beyond the outer walls)"
echo "  level2 box  $B2   (retaining +${MARGIN2} m, inside the outer walls)"
echo "  level3 box  $B3   (retaining -${MARGIN3} m, sand and terrain only)"
echo "=============================================================="

SRC_N=$(python3 -c "
import rosbag
b=rosbag.Bag('$BAG'); i=b.get_type_and_topic_info()[1]
print(max((v.message_count for v in i.values()
           if v.msg_type=='sensor_msgs/PointCloud2'), default=0)); b.close()")
echo "source has $SRC_N clouds; each level should match it"

rosparam set use_sim_time true

for L in $LEVELS; do
    case $L in
        1) B="$B1";; 2) B="$B2";; 3) B="$B3";;
        *) echo "unknown level: $L (expected 1, 2 or 3)" >&2; exit 1;;
    esac
    OUT="${RUN}${TAG}_level${L}.bag"
    if [ -f "$OUT" ]; then
        echo "== $OUT exists, skipping (delete it to rebuild) =="
        continue
    fi

    echo
    echo "-- level$L   box $B"
    python3 "$CROP" --yaw "$YAW" --tf-timeout "$TF_TIMEOUT" --box $B \
        > "${RUN}${TAG}_level${L}_crop.log" 2>&1 &
    CROP_PID=$!
    sleep 3
    kill -0 $CROP_PID 2>/dev/null || {
        echo "crop node died:"; cat "${RUN}${TAG}_level${L}_crop.log"; exit 1; }

    rosbag record /velodyne_points_cropped -O "$OUT" \
        __name:="rec_${RUN}${TAG}_l${L}" > /dev/null 2>&1 &
    sleep 2
    rosbag play --clock --rate "$RATE" "$BAG" "$TF"
    sleep 3
    rosnode kill "/rec_${RUN}${TAG}_l${L}" > /dev/null 2>&1
    kill -INT $CROP_PID 2>/dev/null; wait $CROP_PID 2>/dev/null
    sleep 2
    grep -E "frames processed|points kept" "${RUN}${TAG}_level${L}_crop.log" \
        | tail -2 | sed 's/^/   /'
done

echo
echo "=============================================================="
echo "VERIFICATION"
echo "=============================================================="
python3 - "$BAG" "${RUN}${TAG}" "$LEVELS" <<'PY'
import os, struct, sys
import numpy as np
import rosbag

src, run = sys.argv[1], sys.argv[2]
levels = [int(x) for x in sys.argv[3].split()]


def stats(path):
    b = rosbag.Bag(path)
    i = b.get_type_and_topic_info()[1]
    t = next((k for k, v in i.items()
              if v.msg_type == "sensor_msgs/PointCloud2"), None)
    st, pts = [], []
    if t:
        for _, (dt, data, md5, pos, py), _x in b.read_messages(topics=[t],
                                                              raw=True):
            _, sec, nsec = struct.unpack_from("<III", data, 0)
            st.append(sec + nsec * 1e-9)
        for k, (_, m, _x) in enumerate(b.read_messages(topics=[t])):
            if k % 50 == 0:
                pts.append(m.width * m.height)
            if len(pts) >= 10:
                break
    b.close()
    return np.array(sorted(st)), (float(np.median(pts)) if pts else 0.0)


s_st, s_pts = stats(src)
print("%-22s %7s %9s %8s %9s" % ("bag", "clouds", "pts/scan", "% of L0",
                                 "max gap"))
print("-" * 60)
print("%-22s %7d %9.0f %8s %9s" % ("source (L0)", len(s_st), s_pts, "100", "-"))

bad = False
prev = None
for L in levels:
    p = "%s_level%d.bag" % (run, L)
    if not os.path.exists(p):
        print("%-22s  not built" % p)
        bad = True
        continue
    st, pt = stats(p)
    d = np.diff(st) if len(st) > 1 else np.array([0.0])
    med = float(np.median(d)) if len(d) else 0.0
    mx = float(d.max()) if len(d) else 0.0
    flag = ""
    # A few clouds missing is normal and harmless
    n_missing = len(s_st) - len(st)
    if med > 0 and mx > 3 * med:
        flag += "  <- INTERIOR GAP of %.2fs (nominal %.2fs)" % (mx, med)
        bad = True
    elif n_missing:
        flag += "  <- %d missing at the ends, fine" % n_missing
    # Only meaningful when consecutive levels were built in this run; a
    # single-level rebuild has nothing above it to compare against.
    if prev is not None and pt >= prev and len(levels) > 1:
        flag += "  <- not fewer points than the level above"
        bad = True
    prev = pt
    print("%-22s %7d %9.0f %8.0f %8.3fs%s"
          % (os.path.basename(p), len(st), pt, 100 * pt / max(s_pts, 1), mx,
             flag))

print("")
if bad:
    print("""Fix the flagged items before running the filtering. An interior gap is a run
of consecutive dropped scans, and the odometry cannot track across it -- the
divergence looks exactly like the structure loss being measured.""")
else:
    print("""All levels gap-free and strictly decreasing in points. Any clouds missing
at the ends are fine: they are missing from every level equally.
""")
PY
