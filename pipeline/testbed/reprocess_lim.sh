#!/bin/bash
# reprocess_lim.sh -- rebuild level 3 with a HEIGHT CEILING, for every run
# whose crop geometry is recorded, and regenerate the figures.
#
# This script applies the same ceiling to every run it can, so the comparison
# is made on more than one recording.
#
# Usage:
#   ./reprocess_lim.sh --dry-run          # print the plan, touch nothing
#   ./reprocess_lim.sh --only ellipse1    # one run end to end
#   ./reprocess_lim.sh                    # everything
#
# Needs roscore in another terminal, and LeGO-LOAM not running.
# Allow roughly 25 minutes per run. Resumable: anything already produced is
# skipped, so a failure halfway costs only the step that failed.

set -u

# ---------------------------------------------------------------- config

RUNS=(
"september:2026-09-16-07-03-07.bag:yandiwanba3_tf.bag:29.75:-9.2 1.3 -2.4 16.1:final"
"rectangle:rectangle_2026-10-07-06-53-20.bag:rectangle_tf.bag:33.00:-9.89 0.71 -11.73 6.87"
"ellipse1:ellipse1_2026-10-07-06-56-59.bag:ellipse1_tf.bag:31.00:-10.33 0.17 -1.71 16.89"
"zigzag:zigzag_2026-*.bag:zigzag*tf*.bag:34.50:-9.89 0.61 -0.43 18.17:zigzag"
)

ZLO="${ZLO:--5}"
ZHI="${ZHI:--0.45}"
RATE="${RATE:-0.3}"
TF_TIMEOUT="${TF_TIMEOUT:-2.0}"
CROP="${CROP:-./crop_map_frame.py}"
ABL="${ABL:-./run_ablation.sh}"
BRAKE="${BRAKE:-./brake.sh}"
ROOT="${ROOT:-results/testbed}"
LOG="${LOG:-reprocess_lim_$(date +%Y%m%d_%H%M%S).log}"

DRY=0
ONLY=""
while [ $# -gt 0 ]; do
    case "$1" in
        --dry-run) DRY=1;;
        --only) ONLY="${2:?--only needs a run name}"; shift;;
        *) echo "unknown argument: $1" >&2; exit 1;;
    esac
    shift
done

say() { echo "$@" | tee -a "$LOG"; }
run() {  # echo then execute, unless --dry-run
    say "+ $*"
    [ "$DRY" = 1 ] && return 0
    "$@"
}

# ---------------------------------------------------------------- preflight
say "=============================================================="
say "reprocess_lim   ceiling ZHI=$ZHI   rate ${RATE}x   log $LOG"
say "=============================================================="

fail=0
for f in "$CROP" "$ABL" "$BRAKE"; do
    [ -f "$f" ] || { say "MISSING: $f"; fail=1; }
done
grep -q 'tf-timeout' "$CROP" 2>/dev/null || {
    say "MISSING: $CROP has no --tf-timeout (old copy)"; fail=1; }
rostopic list >/dev/null 2>&1 || {
    say "MISSING: no ROS master -- start roscore in another terminal"; fail=1; }
pgrep -f mapOptmization >/dev/null && {
    say "LeGO-LOAM is running -- stop it, its tf will fight the replay"
    fail=1; }

PLAN=()
for entry in "${RUNS[@]}"; do
    IFS=: read -r NAME BAG TF YAW BOX BASE <<< "$entry"
    BASE="${BASE:-$NAME}"
    [ -n "$ONLY" ] && [ "$ONLY" != "$NAME" ] && continue
    case "$BAG$TF" in *"<"*) say "SKIP $NAME: fill in its bag/tf in RUNS"
                             continue;; esac

    bad=0
    for field in BAG TF; do
        eval "pat=\"\$$field\""
        case "$pat" in
            *[*?]*)
                # shellcheck disable=SC2206
                hits=( $pat )
                [ -e "${hits[0]:-}" ] || hits=()
                if [ "${#hits[@]}" -ne 1 ]; then
                    say "SKIP $NAME: '$pat' matched ${#hits[@]} files, need 1"
                    [ "${#hits[@]}" -gt 1 ] && say "   ${hits[*]}"
                    bad=1; break
                fi
                eval "$field=\"\${hits[0]}\""
                say "   $NAME: $pat -> ${hits[0]}";;
        esac
    done
    [ "$bad" = 1 ] && continue

    [ -f "$BAG" ] || { say "SKIP $NAME: source bag not found: $BAG"; continue; }
    [ -f "$TF"  ] || { say "SKIP $NAME: tf bag not found: $TF"; continue; }
    [ -f "$ROOT/$BASE/est_raw_L0.txt" ] || {
        say "SKIP $NAME: no $ROOT/$BASE/est_raw_L0.txt to measure against"
        continue; }
    # Store the RESOLVED entry, with BASE always present, so the later phases
    # re-parse filenames rather than globs.
    PLAN+=("$NAME:$BAG:$TF:$YAW:$BOX:$BASE")
done

[ ${#PLAN[@]} -eq 0 ] && { say "nothing to do"; exit 1; }
[ "$fail" = 1 ] && { say ""; say "preflight failed -- fix the above"; exit 1; }

say ""
say "will process: ${#PLAN[@]} run(s)"
for entry in "${PLAN[@]}"; do
    IFS=: read -r NAME _B _T YAW BOX BASE <<< "$entry"
    say "   $NAME   yaw $YAW   level-3 box $BOX $ZLO $ZHI   L0 from ${BASE:-$NAME}"
done
say ""
say "FREE SPACE: each cropped bag is roughly 250 MB."
df -h . | tail -1 | tee -a "$LOG"
say ""

# ---------------------------------------------------------------- 1. crop
say "=============================================================="
say "PHASE 1   cropping level 3 with the ceiling"
say "=============================================================="

for entry in "${PLAN[@]}"; do
    IFS=: read -r NAME BAG TF YAW BOX _BASE <<< "$entry"
    OUT="${NAME}_lim_level3.bag"
    if [ -f "$OUT" ]; then say "== $OUT exists, skipping =="; continue; fi

    # Past this point the bag has to be cropped, which needs a real box. A
    # placeholder is only ever legitimate when the crop is already on disk.
    read -r -a BOXA <<< "$BOX"
    if [ "${#BOXA[@]}" -ne 4 ]; then
        say ""
        say "!! $NAME: box is '$BOX', not four numbers, and $OUT is missing."
        say "   Refusing to crop from a guessed box -- the result would not be"
        say "   comparable with ${NAME}_level3. See ZIGZAG at the top."
        continue
    fi

    say ""
    say "-- $NAME   box $BOX $ZLO $ZHI"
    if [ "$DRY" = 1 ]; then
        say "+ (crop node, recorder, rosbag play $BAG $TF)"
        continue
    fi

    python3 "$CROP" --yaw "$YAW" --tf-timeout "$TF_TIMEOUT" \
        --box $BOX "$ZLO" "$ZHI" > "${NAME}_lim_level3_crop.log" 2>&1 &
    CROP_PID=$!
    sleep 3
    kill -0 $CROP_PID 2>/dev/null || {
        say "crop node died:"; tail -20 "${NAME}_lim_level3_crop.log" \
            | tee -a "$LOG"; exit 1; }

    rosbag record /velodyne_points_cropped -O "$OUT" \
        __name:="rec_${NAME}_lim" > /dev/null 2>&1 &
    sleep 2
    rosbag play --clock --rate "$RATE" "$BAG" "$TF" 2>&1 | tail -2 \
        | tee -a "$LOG"
    sleep 3
    rosnode kill "/rec_${NAME}_lim" > /dev/null 2>&1
    kill -INT $CROP_PID 2>/dev/null; wait $CROP_PID 2>/dev/null
    sleep 2
    grep -E "frames processed|points kept" "${NAME}_lim_level3_crop.log" \
        | tail -2 | sed 's/^/   /' | tee -a "$LOG"

    # The ceiling should remove the overhead returns AND NOTHING ELSE. If the
    # points lost are not accounted for by the points above the sensor, the
    # cut is also taking terrain and the comparison is not clean.
    ORIG="${NAME}_level3.bag"
    [ -f "$ORIG" ] || ORIG="level3.bag"   # September's bags have no prefix
    python3 - "$ORIG" "$OUT" 2>&1 <<'PY' | tee -a "$LOG"
import sys, numpy as np, rosbag
from sensor_msgs import point_cloud2 as pc2
def scan(path, n=30):
    zs, k = [], 0
    with rosbag.Bag(path) as b:
        for i, (_t, m, _ts) in enumerate(b.read_messages()):
            p = np.array(list(pc2.read_points(m, field_names=('x','y','z'),
                                              skip_nans=True)))
            zs.append(p[:, 2]); k += 1
            if i >= n: break
    z = np.concatenate(zs)
    return z.size / k, (z > 0).sum() / k, z.max()
try:
    a_pts, a_above, a_max = scan(sys.argv[1])
    b_pts, b_above, b_max = scan(sys.argv[2])
except Exception as e:
    print("   could not compare bags: %s" % e); raise SystemExit
print("   original  %8.0f pts/scan  above sensor %6.1f  zmax %+.2f"
      % (a_pts, a_above, a_max))
print("   ceilinged %8.0f pts/scan  above sensor %6.1f  zmax %+.2f"
      % (b_pts, b_above, b_max))
lost, expected = a_pts - b_pts, a_above - b_above
extra = lost - expected          # removed, but NOT overhead -> terrain
print("   lost %.0f pts/scan, %.0f were above the sensor (%.0f were not)"
      % (lost, expected, extra))
# Two tests, and BOTH must fail before this is worth stopping for.
if b_above > 0:
    print("   !! points remain above the sensor -- ceiling did not apply")
elif extra > 0.25 * max(lost, 1) and extra > 0.002 * a_pts:
    print("   !! the loss is NOT accounted for by overhead points --")
    print("      %.0f pts/scan of terrain went too (%.2f%% of the scan)."
          % (extra, 100.0 * extra / a_pts))
    print("      Raise ZHI.")
else:
    print("   OK: the ceiling took the overhead returns and little else")
PY
done

# ---------------------------------------------------------------- 2. replay
# One build, all runs; then flip once and repeat. Two rebuilds total rather
# than two per run.
say ""
say "=============================================================="
say "PHASE 2   replaying both builds"
say "=============================================================="

do_pass() {   # <brake-args...> <out-suffix> <label>
    local suffix="$1"; shift
    local label="$1"; shift
    say ""
    say "-- build: $label"
    run bash "$BRAKE" "$@" 2>&1 | tail -5 | tee -a "$LOG"
    [ "$DRY" = 0 ] && bash "$BRAKE" status 2>&1 | tee -a "$LOG"

    for entry in "${PLAN[@]}"; do
        IFS=: read -r NAME _B _T _Y _BOX BASE <<< "$entry"
        BASE="${BASE:-$NAME}"
        local src; src=$(echo "$entry" | cut -d: -f2)
        local out="$ROOT/${NAME}_lim${suffix}"
        if [ -f "$out/est_raw_L3.txt" ]; then
            say "== ${NAME}_lim${suffix} done, skipping =="; continue
        fi
        [ -f "${NAME}_lim_level3.bag" ] || {
            say "== ${NAME}_lim_level3.bag missing, skipping =="; continue; }
        say ""
        say "-- $NAME  ($label)"
        RUN="${NAME}_lim" BAG="$src" ONLY=L3 OUT="$out" \
            run bash "$ABL" 2>&1 | tail -12 | tee -a "$LOG"
        # Every measurement is against the run's own unfiltered pass, which
        # is crop-independent -- same recording, replayed uncropped.
        [ "$DRY" = 0 ] && [ -d "$out" ] && \
            cp -n "$ROOT/$BASE/est_raw_L0.txt" "$out/" 2>/dev/null
    done
}

do_pass "_m0f10" "threshold removed (map 0, feature 10)" set 0 10
do_pass ""       "stock (map 100, feature 10)"           on

say ""
run bash "$BRAKE" on > /dev/null 2>&1
say "workspace restored to the stock build"

# ---------------------------------------------------------------- 3. figures
say ""
say "=============================================================="
say "PHASE 3   figures"
say "=============================================================="

DIRS=()
for entry in "${PLAN[@]}"; do
    IFS=: read -r NAME _R <<< "$entry"
    [ -f "$ROOT/${NAME}_lim/est_raw_L3.txt" ] && DIRS+=("$ROOT/${NAME}_lim")
done

if [ ${#DIRS[@]} -eq 0 ]; then
    say "no completed runs to plot"
else
    run python3 brake_effect.py --level L3 "${DIRS[@]}" \
        --plot brake_effect_L3_lim.png 2>&1 | tee -a "$LOG"
    # parked_bars takes its runs POSITIONALLY FIRST: --levels is nargs="+"
    # and will swallow a path that follows it.
    run python3 parked_bars.py "${DIRS[@]}" --levels L3 \
        --plot parked_L3_lim.png 2>&1 | tee -a "$LOG"
    for entry in "${PLAN[@]}"; do
        IFS=: read -r NAME _R <<< "$entry"
        [ -f "$ROOT/${NAME}_lim/est_raw_L3.txt" ] || continue
        run python3 error_growth.py --run "${NAME}_lim" --levels L3 \
            --plot "growth_${NAME}_lim.png" 2>&1 | tee -a "$LOG"
        run python3 error_growth.py --run "${NAME}_lim" --levels L3 --x time \
            --plot "growth_${NAME}_lim_time.png" 2>&1 | tee -a "$LOG"
    done
fi

