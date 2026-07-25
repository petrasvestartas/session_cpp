#!/usr/bin/env bash
# Rotated/randomized chair boolean validation harness (v2 — per-config logs, no interleaving).
#
#   ./validate_rot2.sh <exe> <op> det [cfgs...]     # deterministic configs (default: all 7)
#   ./validate_rot2.sh <exe> <op> rnd <N>           # N seeded random orientations
#
# Per config: OURS (faces/solid/naked/vol + topology_report) | OCCT truth (--<op> A B, cached)
# | OCCT BRepCheck of OUR exported result STEP (VALID + subshape failure count) | verdict.
set -u
EXE="${1:?exe}"; OP="${2:?cut|common|fuse}"; MODE="${3:?det|rnd}"; shift 3
CH="serialization/boolean_steps/chairs"
A="$CH/chair0.stp"
SP="C:/pc/3_code/code_rust/session/validation/step_probe/build/Release/step_probe.exe"
LOGD="${VR_LOGD:-/tmp/vr2_logs}"; mkdir -p "$LOGD"
CACHE="C:/pc/3_code/code_rust/session/validation/rot_truth_${OP}.txt"; touch "$CACHE"

truth_of() {  # $1 = B step path, $2 = cache key -> "vol/faces/solids/valid"
  local hit; hit=$(grep -F "$2	" "$CACHE" 2>/dev/null | head -1 | cut -f2)
  if [ -n "$hit" ]; then echo "$hit"; return; fi
  local t tv tf ts tvd
  t=$("$SP" "--$OP" "$A" "$1" 2>/dev/null)
  tv=$(echo "$t" | awk '/OP_VOLUME/{print $2}'); tf=$(echo "$t" | awk '/OP_FACES/{print $2}')
  ts=$(echo "$t" | awk '/OP_SOLIDS/{print $2}'); tvd=$(echo "$t" | awk '/OP_VALID/{print $2}')
  [ -z "$tv" ] && { echo "n/a"; return; }
  printf '%s\t%s\n' "$2" "$tv/$tf/$ts/v$tvd" >> "$CACHE"
  echo "$tv/$tf/$ts/v$tvd"
}

run_det() {
  local cfgs="$*"; [ -z "$cfgs" ] && cfgs="z15 z30 z45 z90 x20 y30 z30x20"
  for c in $cfgs; do
    SESSION_CHAIRS="$CH" SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY="$c" SESSION_OP="$OP" \
      SESSION_TOPO_CHECK=1 SESSION_ROT_STEP=1 "$EXE" zzzz > "$LOGD/vr_${OP}_$c.txt" 2>&1 &
  done
  wait
  echo "$cfgs"
}

run_rnd() {
  local N="${1:-10}"
  SESSION_CHAIRS="$CH" SESSION_ROT_RANDOM="$N" SESSION_OP="$OP" \
    SESSION_TOPO_CHECK=1 SESSION_ROT_STEP=1 "$EXE" zzzz > "$LOGD/vr_${OP}_rnd.txt" 2>&1
  seq -f "r%03g" 0 $((N-1))
}

if [ "$MODE" = det ]; then CFGS=$(run_det "$@"); SUB=rot; else CFGS=$(run_rnd "${1:-10}"); SUB=rnd; fi

printf "%-8s | %-28s | %-22s | %-26s | %-16s | %s\n" \
  cfg "OURS vol/faces/naked/solid" "TOPO" "OCCT_truth vol/f/sol/valid" "OUR_STEP occt" verdict
for c in $CFGS; do
  if [ "$MODE" = det ]; then LOG="$LOGD/vr_${OP}_$c.txt"; else LOG="$LOGD/vr_${OP}_rnd.txt"; fi
  line=$(grep -E "chairs(ROT|RND) +$c +$OP" "$LOG" | head -1)
  ours=$(echo "$line" | sed -E 's/.*faces ([0-9]+) solid ([0-9]+) naked ([0-9]+) vol ([-0-9.]+).*/\4\/\1\/\3\/\2/')
  topo=$(grep -A1 -E "chairs(ROT|RND) +$c +$OP" "$LOG" | grep -oE "naked=[0-9]+ nonmanifold=[0-9]+ shells=[0-9]+ dupV=[0-9]+ manifold=[A-Z]+" | head -1)
  truth=$(truth_of "$CH/$SUB/B_${c}.step" "$c")
  rstep="$CH/$SUB/res_${c}_${OP}.step"
  ourres="no-step"
  if [ -f "$rstep" ]; then
    rr=$("$SP" "$rstep" 2>/dev/null)
    rv=$(echo "$rr" | awk '/^VOLUME/{printf "%.3f", $2}'); rvd=$(echo "$rr" | awk '/^VALID/{print $2}')
    nfail=$("$SP" "$rstep" -c 2>/dev/null | grep -cE "SOLID|SHELL|FACE|WIRE|EDGE|VERTEX")
    ourres="v${rvd}/${rv}/f${nfail}"
  fi
  solid=$(echo "$ours" | awk -F/ '{print $4}'); naked=$(echo "$ours" | awk -F/ '{print $3}')
  v="OPEN"; [ "$solid" = "1" ] && [ "$naked" = "0" ] && v="SOLID"
  echo "$ourres" | grep -q "^v1/" && v="$v+OCCTVALID"
  printf "%-8s | %-28s | %-22s | %-26s | %-16s | %s\n" "$c" "$ours" "$topo" "$truth" "$ourres" "$v"
done
