#!/bin/bash
# step4_repeats.sh -- run ONE condition N times, unchanged, and measure the
# spread. Then optionally repeat at a slower replay rate to see whether the
# spread is caused by dropped scans.
#
# WHAT THIS SCRIPT ESTABLISHES
#   1. the size of the non-determinism, as a spread over N identical runs
#   2. whether slowing the replay removes it (RATE < 1 gives the pipeline more
#      wall-clock time per scan, so fewer scans are missed)
#   3. how many repeats a real result needs, computed from the observed spread
#
# Until 1 is known, no drift number in this project can be quoted, and none of
# the ablation's drift ordering can be called a result. lambda_min is unaffected
# -- it is a per-scan median and reproduces to ~3%.
#
# Usage:
#     ./step4_repeats.sh                      # 3 repeats, full rate
#     N=3 RATE=0.3 OUT=results/testbed/rep03 ./step4_repeats.sh
#     KF=0.00 N=3 ./step4_repeats.sh          # repeats of the walls-removed case
#
# Run from the directory holding level1.bag, with roscore in another terminal.
# At RATE=0.3 each repeat takes ~5.5 min, so N=3 is ~17 min.

set -u

WS="${WS:-$HOME/catkin_ws}"
BAG="${BAG:-level1.bag}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
FILT="${FILT:-$WS/pipeline/testbed/filter_walls.py}"
DERIVE="${DERIVE:-$HOME/catkin_ws/pipeline/testbed/derive_structure.py}"
TABLE="${TABLE:-./drift_table.py}"

KF="${KF:-1.00}"          # 1.00 = passthrough, the cleanest control
N="${N:-3}"
RATE="${RATE:-1.0}"
OUT="${OUT:-results/testbed/repeats}"
SEED="${SEED:-1}"

MIN_INLIERS="${MIN_INLIERS:-400}"
MAX_PLANES="${MAX_PLANES:-12}"
MIN_RANGE="${MIN_RANGE:-2.0}"
MIN_EXTENT="${MIN_EXTENT:-1.0}"
MIN_Z_SPAN="${MIN_Z_SPAN:-0.20}"
MIN_COL_SPAN="${MIN_COL_SPAN:-0.10}"

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

[ -f "$BAG" ]  || { echo "missing bag: $BAG" >&2; exit 1; }
[ -f "$REC" ]  || { echo "missing record_run.py at: $REC" >&2; exit 1; }
[ -f "$FILT" ] || { echo "missing filter_walls.py at: $FILT" >&2; exit 1; }
grep -q 'def column_span' "$FILT" || {
    echo "ERROR: $FILT is the OLD copy (no column_span)." >&2; exit 1; }

echo "=============================================================="
echo "step4: $N identical repeats"
echo "  keep_fraction $KF      (1.00 = passthrough control)"
echo "  replay rate   $RATE"
echo "  output        $OUT"
echo "=============================================================="
echo "Every repeat is byte-identical in configuration. Any difference in the"
echo "trajectories is non-determinism in the pipeline, not the manipulation."
echo ""

for i in $(seq 1 "$N"); do
    NAME="R$i"
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        echo "== $NAME already done, skipping =="; continue
    fi

    echo
    echo "=============================================================="
    echo "== repeat $i of $N"
    echo "=============================================================="

    # Cold start: timeLastProcessing persists across a replay, and a warm
    # mapOptmization skips its whole body and writes an empty log.
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

    python3 "$FILT" '~input:=/vp_raw' '~output:=/velodyne_points' \
        _keep_fraction:="$KF" _keep_sector_deg:=360 _seed:="$SEED" \
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

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init \
        --note "repeat $i, keep_fraction=$KF, rate=$RATE" \
        > "$OUT/${NAME}_rec.log" 2>&1 &
    REC_PID=$!
    sleep 2

    echo "-- playing $BAG at rate $RATE"
    rosbag play --clock --rate "$RATE" "$BAG" \
        /velodyne_points_cropped:=/vp_raw

    sleep 8
    kill -INT $REC_PID  2>/dev/null; wait $REC_PID  2>/dev/null
    kill -INT $FILT_PID 2>/dev/null; wait $FILT_PID 2>/dev/null
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    sleep 3

    grep "filter_walls summary" "$OUT/${NAME}_filter.log" | tail -1 \
        | sed 's/^/   /' || true
    if [ -f "$OUT/est_raw_$NAME.txt" ]; then
        python3 - "$OUT/est_raw_$NAME.txt" <<'PY'
import numpy as np, sys
a = np.loadtxt(sys.argv[1], ndmin=2)
print("   %d poses over %.1f s  (%.2f Hz)"
      % (len(a), a[-1,0]-a[0,0], len(a)/max(a[-1,0]-a[0,0], 1e-6)))
PY
    fi
done

echo
echo "=============================================================="
echo "structure metrics per repeat"
echo "=============================================================="
for d in "$OUT"/logs/*/; do
    [ -d "$d" ] || continue
    [ -f "$d/structure.csv" ] || python3 "$DERIVE" "$d" >/dev/null 2>&1 \
        || echo "   derive failed for $d"
done

python3 - "$OUT" "$RATE" "$KF" <<'PY'
import glob, os, sys
import numpy as np
import pandas as pd

out, rate, kf = sys.argv[1], sys.argv[2], sys.argv[3]

lam = []
for p in sorted(glob.glob(os.path.join(out, "logs", "*", "structure.csv"))):
    name = os.path.basename(os.path.dirname(p))
    df = pd.read_csv(p)
    if "map_full_lambda_min" in df.columns and not df.empty:
        lam.append((name, len(df), float(df.map_full_lambda_min.median())))

if lam:
    print("%-6s %7s %12s" % ("repeat", "scans", "lambda_min"))
    for n, k, v in lam:
        print("%-6s %7d %12.1f" % (n, k, v))
    v = np.array([x[2] for x in lam])
    if len(v) > 1:
        print("  spread: mean %.1f, sd %.1f (%.1f%% of mean), range %.1f"
              % (v.mean(), v.std(ddof=1), 100*v.std(ddof=1)/v.mean(),
                 v.max()-v.min()))

# ---- coverage, which is itself not repeatable ------------------------
files = sorted(glob.glob(os.path.join(out, "est_raw_R*.txt")))
if len(files) < 2:
    print("\nneed at least 2 repeats to measure spread")
    raise SystemExit

print("\n%-6s %7s %9s %8s" % ("repeat", "poses", "duration", "Hz"))
durs = []
for f in files:
    a = np.loadtxt(f, ndmin=2)
    d = a[-1, 0] - a[0, 0]
    durs.append(d)
    print("%-6s %7d %8.1fs %8.2f" % (os.path.basename(f)[8:-4], len(a), d,
                                     len(a)/max(d, 1e-6)))
if max(durs) - min(durs) > 2.0:
    print("  coverage differs by %.1f s across identical runs -- the pipeline is"
          % (max(durs) - min(durs)))
    print("  losing a variable amount of the recording.")

# ---- pairwise trajectory spread --------------------------------------
runs = []
for f in files:
    a = np.loadtxt(f, ndmin=2)
    runs.append((os.path.basename(f)[8:-4], a[:, 0], a[:, 1:4]))

t0 = max(t[0] for _, t, _ in runs)
t1 = min(t[-1] for _, t, _ in runs)
grid = np.linspace(t0, t1, max(int((t1 - t0) * 5), 8))
P = {n: np.column_stack([np.interp(grid, t, p[:, k]) for k in (0, 1, 2)])
     for n, t, p in runs}

print("\npairwise separation between IDENTICAL runs, over [%.1f, %.1f] s"
      % (0.0, t1 - t0))
print("%-14s %9s %9s %9s" % ("pair", "median", "p95", "final"))
finals, meds = [], []
names = [n for n, _, _ in runs]
for i in range(len(names)):
    for j in range(i + 1, len(names)):
        d = P[names[i]] - P[names[j]]
        dist = np.linalg.norm(d[:, [0, 2]], axis=1)
        print("%-14s %9.3f %9.3f %9.3f"
              % ("%s vs %s" % (names[i], names[j]),
                 np.median(dist), np.percentile(dist, 95), dist[-1]))
        finals.append(dist[-1]); meds.append(np.median(dist))

sd_med = float(np.mean(meds))
print("""
This is the NOISE FLOOR. Nothing smaller than it can be attributed to a
manipulation, at any number of levels.""")
print("mean pairwise median separation: %.3f m" % sd_med)
print("mean pairwise final separation:  %.3f m" % float(np.mean(finals)))

print("\nrate = %s, keep_fraction = %s" % (rate, kf))
if sd_med < 0.10:
    print("""  Small. If this came from RATE < 1, the non-determinism was dropped scans
  and every drift number should be regenerated at this rate -- at which point
  the ablation's drift ordering becomes measurable with few repeats.""")
else:
    print("""  Large. Identical runs disagree by more than this project's between-level
  gaps, so drift needs repeats and error bars regardless of rate.""")
    for delta in (0.5, 1.0, 2.0):
        n = max(int(np.ceil(2 * (1.96 * sd_med / delta) ** 2)), 2)
        print("    to resolve a %.1f m difference: ~%d runs per level" % (delta, n))

print("""
Compare against the ablation's between-level gaps in median divergence:
    L1 0.011 | FIX000 1.301 | L2 2.179 | L3 7.399
A noise floor above ~0.5 m means only L3 is resolvable with one run each.""")
PY

echo
echo "=============================================================="
echo "against the unfiltered baseline, if you have it:"
echo "  python3 $TABLE --baseline results/testbed/levels/est_raw_L0.txt \\"
echo "      $OUT/est_raw_R*.txt"
echo "=============================================================="
