#!/usr/bin/env bash
# OCCT validation harness for rotated-chair Boolean results.
# Compares OUR result (exported STEP) against OCCT ground truth + OCCT BRepCheck.
#
#   ./validate_rot.sh <exe> <mode> <op> [N|cfgs...]
#     mode = det   : deterministic configs (default list or listed cfgs)
#     mode = rnd   : N random configs (SESSION_ROT_RANDOM)
#   Emits a table: cfg | our(vol/faces/naked/solid) | OCCT_truth(vol/faces/solids/valid) |
#                  our_result_STEP OCCT(valid/vol/faces) | verdict
set -u
EXE="${1:?exe}"; MODE="${2:?det|rnd}"; OP="${3:?cut|common|fuse}"; shift 3
CHDIR="serialization/boolean_steps/chairs"
A="$CHDIR/chair0.stp"
SP="C:/pc/3_code/code_rust/session/validation/step_probe/build/Release/step_probe.exe"
OPFLAG="--$OP"

run_our() {  # populates /tmp/vr_out.txt with chairsROT/chairsRND lines + writes res STEP
  if [ "$MODE" = "rnd" ]; then
    local N="${1:-10}"
    SESSION_CHAIRS="$CHDIR" SESSION_ROT_RANDOM="$N" SESSION_OP="$OP" \
      SESSION_NO_MERGE=1 SESSION_ROT_STEP=1 "$EXE" zzzz > /tmp/vr_out.txt 2>&1
  else
    local cfgs="$*"
    [ -z "$cfgs" ] && cfgs="z15 z30 z45 z90 x20 y30 z30x20"
    : > /tmp/vr_out.txt
    for c in $cfgs; do
      SESSION_CHAIRS="$CHDIR" SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY="$c" SESSION_OP="$OP" \
        SESSION_NO_MERGE=1 SESSION_ROT_STEP=1 "$EXE" zzzz >> /tmp/vr_out.txt 2>&1 &
    done
    wait
  fi
}

occt() { "$SP" "$1" 2>/dev/null | awk '/OP_|SOLIDS|FACES|VOLUME|VALID/{print}'; }

echo "=== running our booleans ($MODE $OP) ==="
run_our "$@"

printf "%-8s | %-26s | %-30s | %-24s | %s\n" cfg "OURS vol/f/naked/solid" "OCCT_truth vol/f/solids" "OUR_res OCCT valid/vol/f" verdict
grep -hE "chairs(ROT|RND)" /tmp/vr_out.txt | while read -r line; do
  cfg=$(echo "$line" | awk '{print $2}')
  ours=$(echo "$line" | sed -E 's/.*faces ([0-9]+) solid ([0-9]+) naked ([0-9]+) vol ([-0-9.]+).*/\4\/\1\/\3\/\2/')
  # OCCT ground truth on the rotated B
  bstep="$CHDIR/$([ "$MODE" = rnd ] && echo rnd || echo rot)/B_${cfg}.step"
  truth="n/a"
  if [ -f "$bstep" ]; then
    t=$("$SP" "$OPFLAG" "$A" "$bstep" 2>/dev/null)
    tv=$(echo "$t" | awk '/OP_VOLUME/{print $2}'); tf=$(echo "$t" | awk '/OP_FACES/{print $2}')
    ts=$(echo "$t" | awk '/OP_SOLIDS/{print $2}'); tvd=$(echo "$t" | awk '/OP_VALID/{print $2}')
    truth="${tv}/${tf}/${ts}(v$tvd)"
  fi
  # OCCT check of OUR exported result
  rstep="$CHDIR/$([ "$MODE" = rnd ] && echo rnd || echo rot)/res_${cfg}_${OP}.step"
  ourres="no-step"
  if [ -f "$rstep" ]; then
    rr=$("$SP" "$rstep" 2>/dev/null)
    rv=$(echo "$rr" | awk '/^VOLUME/{print $2}'); rf=$(echo "$rr" | awk '/^FACES/{print $2}')
    rvd=$(echo "$rr" | awk '/^VALID/{print $2}')
    ourres="v${rvd}/${rv}/${rf}"
  fi
  solid=$(echo "$ours" | awk -F/ '{print $4}')
  verdict=$([ "$solid" = 1 ] && echo OURS_SOLID || echo OURS_OPEN)
  printf "%-8s | %-26s | %-30s | %-24s | %s\n" "$cfg" "$ours" "$truth" "$ourres" "$verdict"
done
