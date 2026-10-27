#!/bin/bash
# step3_refilter.sh -- re-run the plane-filter with the FIXED filter.
#
# WHAT IT MEASURES
# Two conditions, both applied to level1.bag (all walls still present):
#
#   KF10  keep_fraction 1.0 -- passthrough. Wall points are identified and then
#         all kept. Nothing should change, so lambda_min MUST reproduce L1
#         (~386). This is the control: if it does not, the filter is perturbing
#         the cloud through some route other than wall removal -- field loss,
#         dropped scans, timing -- and no other number from this run is usable.
#
#   KF00  keep_fraction 0.0 -- every detected wall point removed. Compare its
#         lambda_min against L1 to get the true cost of removing the walls the
#         sensor can actually resolve as walls.
#
# Interpretation is limited by the range and resolution of wall detection. 
# Filtered results cannot establish the contribution of walls that were not successfully detected.
#
# Usage:
#     ./step3_refilter.sh                     # the two decisive conditions
#     FRACS="1.00 0.60 0.35 0.20 0.10 0.00" ./step3_refilter.sh    # full sweep
#
# Run from the directory holding level1.bag. Needs roscore in another terminal.

set -u

WS="${WS:-$HOME/catkin_ws}"
BAG="${BAG:-level1.bag}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
FILT="${FILT:-$WS/pipeline/testbed/filter_walls.py}"
DERIVE="${DERIVE:-$HOME/catkin_ws/pipeline/testbed/derive_structure.py}"
OUT="${OUT:-results/testbed/refilter}"
SEED="${SEED:-1}"

# Shape-test settings. Defaults in the script match these; they are passed
# explicitly so the run is self-documenting and reproducible from the log.
MIN_INLIERS="${MIN_INLIERS:-400}"
MAX_PLANES="${MAX_PLANES:-12}"
MIN_RANGE="${MIN_RANGE:-2.0}"
MIN_EXTENT="${MIN_EXTENT:-1.0}"
MIN_Z_SPAN="${MIN_Z_SPAN:-0.20}"
MIN_COL_SPAN="${MIN_COL_SPAN:-0.10}"

FRACS="${FRACS:-1.00 0.00}"

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

[ -f "$BAG" ]  || { echo "missing bag: $BAG" >&2; exit 1; }
[ -f "$REC" ]  || { echo "missing record_run.py at: $REC" >&2; exit 1; }
[ -f "$FILT" ] || { echo "missing filter_walls.py at: $FILT" >&2
                    echo "find it with: find $WS -name filter_walls.py" >&2
                    exit 1; }

# Refuse to run against the old filter -- that is the whole point of this script.
grep -q 'def column_span' "$FILT" || {
    echo "ERROR: $FILT is the OLD copy (no column_span)." >&2
    echo "Install the fixed filter_walls.py first, then re-run." >&2
    exit 1; }

echo "=============================================================="
echo "step3: re-measuring the plane-filter ablation"
echo "  filter      $FILT"
echo "  shape tests min_extent=$MIN_EXTENT min_z_span=$MIN_Z_SPAN"
echo "              min_col_span=$MIN_COL_SPAN min_range=$MIN_RANGE"
echo "  ransac      min_inliers=$MIN_INLIERS max_planes=$MAX_PLANES seed=$SEED"
echo "  conditions  $FRACS"
echo "=============================================================="

for KF in $FRACS; do
    NAME="FIX$(echo "$KF" | tr -d '.')"
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        echo "== $NAME already done, skipping =="; continue
    fi

    echo
    echo "=============================================================="
    echo "== $NAME   keep_fraction=$KF"
    echo "=============================================================="

    # Cold start. mapOptmization's timeLastProcessing persists across a replay,
    # so a warm node silently skips its whole body and writes an empty log.
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
    python3 "$FILT" '~input:=/vp_raw' '~output:=/velodyne_points' \
        _keep_fraction:="$KF" _keep_sector_deg:=360 _seed:="$SEED" \
        _min_inliers:="$MIN_INLIERS" _max_planes:="$MAX_PLANES" \
        _min_range:="$MIN_RANGE" _min_extent:="$MIN_EXTENT" \
        _min_z_span:="$MIN_Z_SPAN" _min_col_span:="$MIN_COL_SPAN" \
        _debug:=true _publish_debug:=false \
        > "$OUT/${NAME}_filter.log" 2>&1 &
    FILT_PID=$!
    sleep 3
    kill -0 $FILT_PID 2>/dev/null || {
        echo "filter_walls died on startup:"; cat "$OUT/${NAME}_filter.log"
        kill -INT $LAUNCH_PID 2>/dev/null; exit 1; }

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init \
        --note "fixed filter, keep_fraction=$KF, min_col_span=$MIN_COL_SPAN" \
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
    grep -E "filter_walls (summary|shape tests)" "$OUT/${NAME}_filter.log" \
        | tail -2 || echo "   (no summary -- check the topic remap)"
done

echo
echo "=============================================================="
echo "deriving structure metrics"
echo "=============================================================="
for d in "$OUT"/logs/*/; do
    [ -d "$d" ] || continue
    [ -f "$d/structure.csv" ] && continue
    echo "-- $d"
    python3 "$DERIVE" "$d" || echo "   derive failed for $d"
done

echo
echo "=============================================================="
echo "RESULT"
echo "=============================================================="
python3 - "$OUT" <<'PY'
import glob, os, sys
import pandas as pd

out = sys.argv[1]
rows = []
for p in sorted(glob.glob(os.path.join(out, "logs", "*", "structure.csv"))):
    name = os.path.basename(os.path.dirname(p))
    df = pd.read_csv(p)
    if "map_full_lambda_min" not in df.columns or df.empty:
        print("%-8s  structure.csv has no usable rows" % name)
        continue
    lam = df.map_full_lambda_min
    deg = df.get("is_degenerate")
    rows.append((name, len(df), float(lam.median()),
                 float(lam.quantile(.25)), float(lam.quantile(.75)),
                 float(deg.mean()) * 100 if deg is not None else float("nan")))

if not rows:
    print("nothing derived -- check the logs above")
    raise SystemExit

print("%-8s %7s %11s %19s %9s" %
      ("cond", "scans", "lambda_min", "IQR", "degen %"))
for n, k, med, q1, q3, dg in rows:
    print("%-8s %7d %11.1f   %8.1f - %-8.1f %8.1f" % (n, k, med, q1, q3, dg))

ref = {"FIX100": None, "FIX000": None}
for n, k, med, *_ in rows:
    if n in ref:
        ref[n] = med

print("""
Reference values from the box-crop levels:
    L1 (inside outer walls)     386.2
    L2 (inside retaining wall)   29.4""")

c, t = ref["FIX100"], ref["FIX000"]
if c is not None:
    err = abs(c - 386.2) / 386.2 * 100
    print("\nCONTROL  FIX100 = %.1f vs L1 386.2  (%.1f%% off)" % (c, err))
    if err > 10:
        print("  FAILED. The passthrough condition should reproduce L1. Something")
        print("  other than wall removal is changing the cloud -- check the filter")
        print("  log for dropped scans, and that `ring` survives and is_dense is")
        print("  true. Do NOT interpret FIX000 until this passes.")
    else:
        print("  passes -- the filter is transparent when the dial is fully open.")

if c is not None and t is not None and abs(c - 386.2) / 386.2 <= 0.10:
    drop = (c - t) / c * 100
    print("\nEFFECT   removing resolvable walls: %.1f -> %.1f  (%.1f%% drop)"
          % (c, t, drop))
    if t <= 60:
        print("  Lands at or below L2. Plane filtering reproduces the box crop:")
        print("  the walls the sensor resolves are the ones carrying the")
        print("  conditioning. The earlier 0.7%% was an artefact of the old filter.")
    elif drop < 5:
        print("  Still a small effect. Now defensible -- but state it as 'the walls")
        print("  plane detection can RESOLVE contribute little', with the ~10 m")
        print("  detection limit attached. The box crop remains the reference for")
        print("  total wall removal, and it gave a 23%% drop at L1 and 93%% at L2.")
    else:
        print("  Intermediate. Between the old 0.7%% and the box crop's L1->L2 step,")
        print("  which is what you would expect if plane detection removes the near")
        print("  walls and misses the far ones.")
PY

echo
echo "Then, for the trajectory side:"
echo "  python3 level_drift.py $OUT/est_raw_FIX100.txt $OUT/est_raw_FIX000.txt"
echo "=============================================================="
