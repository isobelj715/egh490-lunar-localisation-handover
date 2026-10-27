#!/bin/bash
# step6_nobrake.sh check whether drift is caused by missing structure, or by
# LeGO-LOAM's own degeneracy brake
#
# Usage:
#     ./step6_nobrake.sh                 # threshold 0, L0 L2 L3, ~30 min
#     THRESH=25 CONDS="L2" ./step6_nobrake.sh
#
# Needs roscore in another terminal.

set -u

WS="${WS:-$HOME/catkin_ws}"
SRC="${SRC:-$WS/src/LeGO-LOAM/LeGO-LOAM/src/mapOptmization.cpp}"
REC="${REC:-$HOME/catkin_ws/pipeline/testbed/record_run.py}"
DERIVE="${DERIVE:-$HOME/catkin_ws/pipeline/testbed/derive_structure.py}"
RATE="${RATE:-0.3}"
THRESH="${THRESH:-0}"
OUT="${OUT:-results/testbed/nobrake}"
STOCK="${STOCK:-results/testbed/final}"
CONDS="${CONDS:-L0 L2 L3}"
L0BAG="${L0BAG:-2026-09-16-07-03-07.bag}"

BACKUP="${SRC}.stock_backup"

[ -f "$SRC" ] || { echo "cannot find mapOptmization.cpp at $SRC" >&2; exit 1; }
[ -f "$REC" ] || { echo "missing record_run.py at $REC" >&2; exit 1; }
rostopic list >/dev/null 2>&1 || {
    echo "ERROR: no ROS master. Start 'roscore' in another terminal." >&2
    exit 1; }

grep -q 'float eignThre\[6\] = {100, 100, 100, 100, 100, 100};' "$SRC" || {
    echo "ERROR: $SRC does not contain the stock threshold line." >&2
    echo "Either it is already patched, or the formatting differs. Check:" >&2
    grep -n "eignThre" "$SRC" >&2
    exit 1; }

# ---- restore on ANY exit --------------------------------------------
restore() {
    if [ -f "$BACKUP" ]; then
        echo
        echo "=============================================================="
        echo "restoring stock source and rebuilding"
        echo "=============================================================="
        cp -f "$BACKUP" "$SRC" && rm -f "$BACKUP"
        ( cd "$WS" && catkin_make -j2 >/tmp/restore_build.log 2>&1 )
        if grep -q 'float eignThre\[6\] = {100, 100, 100, 100, 100, 100};' "$SRC"; then
            echo "source restored to stock (threshold 100)"
        else
            echo "!!! SOURCE NOT RESTORED -- fix $SRC by hand before any other run"
        fi
        if [ -f "$WS/devel/lib/lego_loam/mapOptmization" ]; then
            echo "rebuilt. Confirm with a stock re-run before trusting later results."
        else
            echo "!!! REBUILD FAILED -- see /tmp/restore_build.log"
        fi
    fi
}
trap restore EXIT INT TERM

cp -f "$SRC" "$BACKUP"

echo "=============================================================="
echo "step6: degeneracy brake OFF (eignThre $THRESH, stock is 100)"
echo "  conditions $CONDS   rate $RATE   out $OUT"
echo "  stock results for comparison: $STOCK"
echo "=============================================================="

sed -i "s/float eignThre\[6\] = {100, 100, 100, 100, 100, 100};/float eignThre[6] = {$THRESH, $THRESH, $THRESH, $THRESH, $THRESH, $THRESH};/" "$SRC"
grep -n "eignThre\[6\]" "$SRC" | sed 's/^/  now: /'

echo
echo "-- rebuilding (this is the modified build) --"
( cd "$WS" && catkin_make -j2 >/tmp/nobrake_build.log 2>&1 ) || {
    echo "BUILD FAILED -- see /tmp/nobrake_build.log"; tail -20 /tmp/nobrake_build.log
    exit 1; }
echo "   built"

source "$WS/devel/setup.bash"
mkdir -p "$OUT"

bag_for() {
    case "$1" in
        L0) echo "$L0BAG";;
        L1) echo "level1.bag";;
        L2) echo "level2.bag";;
        L3) echo "level3.bag";;
        *)  echo "";;
    esac
}

for NAME in $CONDS; do
    BAG=$(bag_for "$NAME")
    [ -n "$BAG" ] && [ -f "$BAG" ] || { echo "no bag for $NAME -- skipping"; continue; }
    [ -f "$OUT/est_raw_$NAME.txt" ] && { echo "== $NAME done, skipping =="; continue; }

    echo
    echo "=============================================================="
    echo "== $NAME   $BAG   (brake off)"
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
        rostopic list 2>/dev/null | grep -q /aft_mapped_to_init && break; sleep 1
    done
    sleep 2

    python3 "$REC" "$NAME" --outdir "$OUT" --force \
        --est-topic /aft_mapped_to_init --note "eignThre=$THRESH rate=$RATE" \
        > "$OUT/${NAME}_rec.log" 2>&1 &
    REC_PID=$!
    sleep 2

    if [ "$NAME" = "L0" ]; then
        rosbag play --clock --rate "$RATE" "$BAG"
    else
        rosbag play --clock --rate "$RATE" "$BAG" \
            /velodyne_points_cropped:=/velodyne_points
    fi

    sleep 8
    kill -INT $REC_PID 2>/dev/null; wait $REC_PID 2>/dev/null
    kill -INT $LAUNCH_PID 2>/dev/null; wait $LAUNCH_PID 2>/dev/null
    sleep 3
done

echo
echo "=============================================================="
echo "structure metrics"
echo "=============================================================="
for d in "$OUT"/logs/*/; do
    [ -d "$d" ] || continue
    [ -f "$d/structure.csv" ] || python3 "$DERIVE" "$d" >/dev/null 2>&1 \
        || echo "   derive failed for $d"
done

python3 - "$OUT" "$STOCK" "$THRESH" <<'PY'
import glob, os, sys
import numpy as np

out, stock, thresh = sys.argv[1], sys.argv[2], sys.argv[3]

try:
    import pandas as pd
except ImportError:
    pd = None


def lam(root, name):
    p = os.path.join(root, "logs", name, "structure.csv")
    if pd is None or not os.path.exists(p):
        return None, None
    d = pd.read_csv(p)
    if "map_full_lambda_min" not in d.columns or d.empty:
        return None, None
    g = d.get("is_degenerate")
    return float(d.map_full_lambda_min.median()), (
        float(g.mean()) * 100 if g is not None else float("nan"))


def traj(root, name):
    p = os.path.join(root, "est_raw_%s.txt" % name)
    if not os.path.exists(p):
        return None
    a = np.loadtxt(p, ndmin=2)
    return a[:, 0], a[:, 1:4]


def plane(p):
    return tuple(i for i in range(3) if i != int(np.argmin(np.ptp(p, axis=0))))


def pathlen(p, pl):
    q = p[:, list(pl)]
    return float(np.linalg.norm(np.diff(q, axis=0), axis=1).sum())


names = sorted({os.path.basename(os.path.dirname(p))
                for p in glob.glob(os.path.join(out, "logs", "*", "structure.csv"))})

print("\n%-6s %22s %22s" % ("", "STOCK (thresh 100)", "NO BRAKE (thresh %s)" % thresh))
print("%-6s %10s %10s %10s %10s" % ("cond", "lambda", "degen%", "lambda", "degen%"))
print("-" * 52)
for n in names:
    ls, ds = lam(stock, n)
    lb, db = lam(out, n)
    print("%-6s %10s %10s %10s %10s"
          % (n,
             "--" if ls is None else "%.1f" % ls,
             "--" if ds is None else "%.0f" % ds,
             "--" if lb is None else "%.1f" % lb,
             "--" if db is None else "%.0f" % db))

# ---- the L0 sanity check ------------------------------------------
s0, b0 = traj(stock, "L0"), traj(out, "L0")
if s0 and b0:
    pl = plane(s0[1])
    t0 = max(s0[0][0], b0[0][0]); t1 = min(s0[0][-1], b0[0][-1])
    g = np.linspace(t0, t1, 400)
    A = np.column_stack([np.interp(g, s0[0], s0[1][:, k]) for k in pl])
    B = np.column_stack([np.interp(g, b0[0], b0[1][:, k]) for k in pl])
    d = np.linalg.norm(A - B, axis=1)
    print("\nSANITY  L0 stock vs L0 no-brake: median %.4f m, max %.4f m"
          % (np.median(d), d.max()))
    if d.max() < 0.10:
        print("  As expected -- L0 never trips the brake at any threshold, so")
        print("  the baseline is unchanged and the comparison below is valid.")
    else:
        print("  UNEXPECTED. L0 should be unaffected by the threshold. Something")
        print("  else differs between the builds -- do not trust the rows below.")

# ---- the actual question ------------------------------------------
if s0:
    pl = plane(s0[1])
    base_len = pathlen(s0[1], pl)
    print("\npath length over the run  (baseline L0 stock = %.2f m)" % base_len)
    print("%-6s %12s %12s %10s" % ("cond", "stock", "no brake", "recovered"))
    print("-" * 44)
    for n in names:
        if n == "L0":
            continue
        a, b = traj(stock, n), traj(out, n)
        if not a or not b:
            continue
        la, lb_ = pathlen(a[1], pl), pathlen(b[1], pl)
        gap = base_len - la
        rec = (la - lb_) / gap * -100 if abs(gap) > 1e-6 else float("nan")
        print("%-6s %9.2f m %9.2f m %9.0f%%" % (n, la, lb_, rec))
    print("""
"recovered" is how much of the stock shortfall against the baseline the
no-brake run closes. Near 100% means the brake was suppressing real motion --
your hypothesis. Near 0% means the optimisation under-moves by itself and the
brake was not the cause.

Path length alone does not settle it. A pose chasing noise also travels
further, so check position error too:""")
    cmd = " ".join("%s/est_raw_%s.txt" % (out, n) for n in names if n != "L0")
    print("\n  python3 drift_table.py --baseline %s/est_raw_L0.txt %s \\"
          % (stock, cmd))
    print("      --plot nobrake.png")
    print("""
If path length recovers but position error gets worse, the brake was trading
motion accuracy for positional stability""")
PY
