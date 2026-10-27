#!/bin/bash
# threshold_experiment.sh -- run one ablation level at several degeneracy
# threshold settings, unattended, and summarise the result.
#
# TWO EXPERIMENTS, ONE SCRIPT
#
# A) WHICH STAGE MATTERS.  './brake.sh off' zeroes the threshold in BOTH
#    mapOptmization.cpp (scan-to-map refinement) and featureAssociation.cpp
#    (scan-to-scan odometry). 
#        CONFIGS="100:10 0:10 100:0 0:0"
#    stock, map-only unbraked, odometry-only unbraked, both unbraked.
#
# B) WHAT THE VALUE SHOULD BE.  Once A says which stage does the work, sweep
#    that stage:
#        CONFIGS="100:10 50:10 25:10 10:10 5:10 0:10"
#    This turns "turn it off" into "set it to X", which is a recommendation
#    rather than a demonstration.
#
# Usage:
#   RUN=rectangle BAG=rectangle_2026-10-07-06-53-20.bag \
#   CONFIGS="100:10 0:10 100:0 0:0" ./threshold_experiment.sh
#
#   RUN=rectangle BAG=... LEVELS="L2 L3" CONFIGS="100:10 0:10" ./threshold_experiment.sh
#
# Needs roscore in another terminal. Allow roughly 10 min per configuration
# (a rebuild plus one 0.3x replay).

set -u

RUN="${RUN:?set RUN, e.g. RUN=rectangle}"
BAG="${BAG:?set BAG, the uncropped source recording}"
CONFIGS="${CONFIGS:-100:10 0:10 100:0 0:0}"
LEVELS="${LEVELS:-L2}"
BASE="${BASE:-results/testbed/$RUN/est_raw_L0.txt}"
ROOT="${ROOT:-results/testbed}"
BRAKE="${BRAKE:-./brake.sh}"
ABL="${ABL:-./run_ablation.sh}"

for f in "$BRAKE" "$ABL"; do
    [ -x "$f" ] || [ -f "$f" ] || { echo "missing: $f" >&2; exit 1; }
done
grep -q 'brake.sh set' "$BRAKE" || {
    echo "ERROR: $BRAKE is the old copy without 'set'. Without it both" >&2
    echo "stages change together and the experiment cannot separate them." >&2
    exit 1; }
rostopic list >/dev/null 2>&1 || {
    echo "ERROR: no ROS master. Start 'roscore' in another terminal." >&2
    exit 1; }
[ -f "$BASE" ] || {
    echo "ERROR: baseline $BASE not found. The unfiltered L0 run is the" >&2
    echo "reference every configuration is measured against." >&2
    exit 1; }

# The stock build must come back even if something fails halfway 
restore() {
    echo
    echo "=============================================================="
    echo "restoring the stock build"
    echo "=============================================================="
    bash "$BRAKE" on || echo "   !! RESTORE FAILED -- run './brake.sh on' yourself" >&2
}
trap restore EXIT INT TERM

echo "=============================================================="
echo "threshold experiment on $RUN"
echo "  levels   $LEVELS"
echo "  configs  $CONFIGS      (mapOptmization:featureAssociation)"
echo "  baseline $BASE"
echo "=============================================================="
START=$(date +%s)

DIRS=""
for CFG in $CONFIGS; do
    M="${CFG%%:*}"; F="${CFG##*:}"
    TAG="m${M}f${F}"
    OUT="$ROOT/${RUN}_$TAG"
    DIRS="$DIRS $OUT"

    DONE=1
    for L in $LEVELS; do
        [ -f "$OUT/est_raw_$L.txt" ] || DONE=0
    done
    if [ "$DONE" = "1" ]; then
        echo
        echo "== $TAG already has every level, skipping =="
        continue
    fi

    echo
    echo "=============================================================="
    echo "== configuration $TAG   map=$M  feature=$F"
    echo "=============================================================="
    bash "$BRAKE" set "$M" "$F" || {
        echo "build failed for $TAG -- skipping" >&2; continue; }

    OUT="$OUT" RUN="$RUN" BAG="$BAG" ONLY="$LEVELS" bash "$ABL" || {
        echo "ablation failed for $TAG" >&2; continue; }
done

echo
echo "total elapsed $(( ($(date +%s) - START) / 60 )) min"

echo
echo "=============================================================="
echo "SUMMARY"
echo "=============================================================="
python3 - "$BASE" "$LEVELS" $DIRS <<'PY'
import os, sys
import numpy as np

base, levels = sys.argv[1], sys.argv[2].split()
dirs = sys.argv[3:]

b = np.loadtxt(base, ndmin=2)
bt, bp = b[:, 0], b[:, 1:4]
plane = [i for i in range(3) if i != int(np.argmin(np.ptp(bp, axis=0)))]
bxy = bp[:, plane]


def stats(path):
    a = np.loadtxt(path, ndmin=2)
    t, p = a[:, 0], a[:, 1:4][:, plane]
    ref = np.column_stack([np.interp(t, bt, bxy[:, k]) for k in (0, 1)])
    d = np.linalg.norm(p - ref, axis=1)
    path_len = float(np.linalg.norm(np.diff(p, axis=0), axis=1).sum())
    true_len = float(np.linalg.norm(np.diff(bxy, axis=0), axis=1).sum())
    # Closest the run comes back to its own start after 10 m of driving: the
    # closure, which needs no baseline at all.
    s = np.r_[0.0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    far = np.flatnonzero(s > 10.0)
    clo = (float(np.min(np.linalg.norm(p[far] - p[0], axis=1)))
           if len(far) else float("nan"))
    return (float(np.median(d)), float(np.percentile(d, 95)), float(d[-1]),
            path_len, 100 * (path_len - true_len) / max(true_len, 1e-9), clo)


hdr = ("%-10s %-6s %9s %9s %9s %9s %9s"
       % ("config", "level", "median", "p95", "final", "path err", "closure"))
print(hdr)
print("-" * len(hdr))
rows = []
for d in dirs:
    tag = os.path.basename(d).split("_")[-1]
    for L in levels:
        p = os.path.join(d, "est_raw_%s.txt" % L)
        if not os.path.exists(p):
            print("%-10s %-6s   (not run)" % (tag, L))
            continue
        med, p95, fin, pl, pe, clo = stats(p)
        rows.append((tag, L, fin, clo))
        print("%-10s %-6s %8.3f %9.3f %9.3f %8.1f%% %9.3f"
              % (tag, L, med, p95, fin, pe, clo))

print("""
config    mapOptmization:featureAssociation thresholds, as mXfY
final     separation from the unfiltered baseline at the end of the run
path err  distance travelled vs the baseline. Negative means the pose is
          under-estimating motion, which is what a suppressed update does.
closure   closest the run returns to its own start after 10 m. NO baseline is
          involved, so this is the number to trust if the two disagree.""")

by = {}
for tag, L, fin, clo in rows:
    by.setdefault(L, []).append((tag, fin, clo))
for L, rs in by.items():
    if len(rs) < 2:
        continue
    rs.sort(key=lambda r: r[1])
    print("\n%s, best to worst by final error:" % L)
    for tag, fin, clo in rs:
        print("   %-10s %8.3f m   (closure %.3f m)" % (tag, fin, clo))
    tags = {t for t, _f, _c in rs}
    if {"m100f10", "m0f10", "m100f0", "m0f0"} <= tags:
        g = {t: f for t, f, _c in rs}
        print("""
WHICH STAGE: compare m0f10 (map refinement unbraked) and m100f0 (odometry
unbraked) against m100f10 (stock, %.3f m) and m0f0 (both, %.3f m).
   map stage alone     %.3f m
   odometry alone      %.3f m
If one of those is close to m0f0 and the other close to stock, that stage is
where the effect lives. If neither helps alone but both together do, the two
suppressions compound and the finding is about the pair."""
              % (g["m100f10"], g["m0f0"], g["m0f10"], g["m100f0"]))
PY
