#!/bin/bash
# brake.sh -- switch LeGO-LOAM's degeneracy brake on or off
#
# WHAT THE BRAKE IS
# mapOptmization and featureAssociation each eigen-decompose their information
# matrix and compare the eigenvalues against a hard-coded threshold:
#
#   mapOptmization.cpp      float eignThre[6] = {100, 100, 100, 100, 100, 100}
#   featureAssociation.cpp  float eignThre[3] = {10, 10, 10}
#
# Any eigenvalue below its threshold has its eigenvector ZEROED in the pose
# update, so the solver refuses to move along directions it considers weakly
# observed. The thresholds are absolute numbers with no stated derivation, and
# they were chosen on terrestrial data. Setting them to 0 disables the brake
# entirely -- every direction gets updated, however weakly observed.
#
# Usage:
#   ./brake.sh status           # what is in the source, and is the binary current
#   ./brake.sh off              # both thresholds -> 0, rebuild  (the FIX000 build)
#   ./brake.sh on               # -> 100 and 10, rebuild  (stock)
#   ./brake.sh set <map> <fea>  # the two stages independently
#
# TWO STAGES, TWO THRESHOLDS
# 'off' zeroes the threshold in mapOptmization.cpp AND in
# featureAssociation.cpp. Those are different stages of the pipeline -- scan-to-
# scan odometry and scan-to-map refinement -- so a result from 'off' cannot say
# which of them caused it. 'set' changes them separately:
#   set 100 10   stock
#   set 0 10     map refinement unbraked, odometry stock
#   set 100 0    odometry unbraked, map refinement stock
#   set 0 0      both unbraked, identical to 'off'

set -u

CMD="${1:-status}"
WS="${WS:-$HOME/catkin_ws}"
SRC="${SRC:-$WS/src}"
BIN="${BIN:-$WS/devel/lib/lego_loam/mapOptmization}"

MAPF=$(find "$SRC" -name mapOptmization.cpp 2>/dev/null | head -1)
FEAF=$(find "$SRC" -name featureAssociation.cpp 2>/dev/null | head -1)

[ -n "$MAPF" ] || { echo "cannot find mapOptmization.cpp under $SRC" >&2; exit 1; }
[ -n "$FEAF" ] || { echo "cannot find featureAssociation.cpp under $SRC" >&2; exit 1; }

show() {
    python3 - "$MAPF" "$FEAF" "$BIN" <<'PY'
import os, re, sys, time

bin_path = sys.argv[3]
newest_src = 0.0
print("")
for p in sys.argv[1:3]:
    m = re.findall(r"eignThre\s*\[\s*(\d+)\s*\]\s*=\s*\{([^}]*)\}", open(p).read())
    newest_src = max(newest_src, os.path.getmtime(p))
    print("%s" % p)
    if not m:
        print("   no eignThre declaration found -- has this file been changed?")
    for n, vals in m:
        v = [x.strip() for x in vals.split(",")]
        state = ("BRAKE OFF" if all(float(x) == 0 for x in v)
                 else "brake on")
        print("   eignThre[%s] = {%s}   <- %s" % (n, ", ".join(v), state))
    print("   last edited  %s"
          % time.strftime("%Y-%m-%d %H:%M", time.localtime(os.path.getmtime(p))))

print("")
if not os.path.exists(bin_path):
    print("BINARY NOT BUILT: %s" % bin_path)
    print("Run './brake.sh on' or './brake.sh off' to build it.")
    sys.exit(2)

b = os.path.getmtime(bin_path)
print("binary       %s" % bin_path)
print("   built     %s" % time.strftime("%Y-%m-%d %H:%M", time.localtime(b)))
if b < newest_src:
    print("")
    print("   !! THE BINARY IS OLDER THAN THE SOURCE.")
    print("   !! The values printed above are NOT what is running")
    print("   !! Rebuild before running anything: ./brake.sh on   (or off)")
    sys.exit(3)
print("   the binary is newer than both sources, so these values are compiled in")
PY
    return $?
}

case "$CMD" in
  status)
    show
    exit $?
    ;;
  off|on|set)
    if [ "$CMD" = "off" ]; then
        TARGET_MAP=0; TARGET_FEA=0
        LABEL="BRAKE OFF (FIX000) -- no eigenvector is ever suppressed"
    elif [ "$CMD" = "on" ]; then
        TARGET_MAP=100; TARGET_FEA=10
        LABEL="brake on (stock) -- LeGO-LOAM's shipped thresholds"
    else
        # Set the two stages independently. They are separate thresholds in
        # separate files and 'off' changes BOTH, so a result obtained that way
        # cannot say which stage produced it. This is how you find out.
        TARGET_MAP="${2:?usage: ./brake.sh set <mapOptmization> <featureAssociation>}"
        TARGET_FEA="${3:?usage: ./brake.sh set <mapOptmization> <featureAssociation>}"
        LABEL="mapOptmization=$TARGET_MAP  featureAssociation=$TARGET_FEA"
    fi

    python3 - "$MAPF" "$TARGET_MAP" <<'PY' || exit 1
import re, sys
p, t = sys.argv[1], sys.argv[2]
s = open(p).read()
if not re.search(r"eignThre\s*\[", s):
    sys.exit("no eignThre in %s" % p)
open(p + ".brakebak", "w").write(s)


def sub(m):
    n = int(m.group(1))
    return "eignThre[%d] = {%s}" % (n, ", ".join([t] * n))


s2, k = re.subn(r"eignThre\s*\[\s*(\d+)\s*\]\s*=\s*\{[^}]*\}", sub, s)
open(p, "w").write(s2)
print("   %s: %d declaration(s) set to %s" % (p.split("/")[-1], k, t))
PY

    python3 - "$FEAF" "$TARGET_FEA" <<'PY' || exit 1
import re, sys
p, t = sys.argv[1], sys.argv[2]
s = open(p).read()
if not re.search(r"eignThre\s*\[", s):
    sys.exit("no eignThre in %s" % p)
open(p + ".brakebak", "w").write(s)


def sub(m):
    n = int(m.group(1))
    return "eignThre[%d] = {%s}" % (n, ", ".join([t] * n))


s2, k = re.subn(r"eignThre\s*\[\s*(\d+)\s*\]\s*=\s*\{[^}]*\}", sub, s)
open(p, "w").write(s2)
print("   %s: %d declaration(s) set to %s" % (p.split("/")[-1], k, t))
PY

    echo ""
    echo "building ($LABEL)"
    echo "   backups at *.brakebak"
    ( cd "$WS" && catkin_make 2>&1 | tail -4 ) || {
        echo "BUILD FAILED -- the binary still holds the PREVIOUS setting." >&2
        echo "Do not run an ablation until this builds." >&2
        exit 1; }

    echo ""
    show
    rc=$?
    if [ $rc -eq 0 ]; then
        echo ""
        echo "=============================================================="
        echo "$LABEL"
        echo "=============================================================="
        if [ "$CMD" = "off" ]; then
            echo "Send these results somewhere separate, e.g."
            echo "  OUT=results/testbed/<run>_nobrake RUN=<run> BAG=<bag> \\"
            echo "      ONLY=\"L2 L3\" ./run_ablation.sh"
            echo ""
            echo "and run './brake.sh on' before any further stock runs."
        fi
    fi
    exit $rc
    ;;
  *)
    echo "usage: ./brake.sh [status|off|on]" >&2
    echo "       ./brake.sh set <mapOptmization> <featureAssociation>" >&2
    echo "         e.g. ./brake.sh set 0 10   (map stage off, odometry stock)" >&2
    exit 1
    ;;
esac
