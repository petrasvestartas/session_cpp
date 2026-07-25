#!/usr/bin/env bash
# ORACLE-GATED per-config validity for the rotated-chair booleans. The internal is_solid (2-trim-only)
# FALSE-POSITIVES on fuzzy over-merge (disconnected/wrong-volume results read "solid"). This gates on
# OCCT ground truth instead: our exported result is BRepChecked by OCCT, and its volume is compared to
# OCCT's OWN boolean result. A config PASSES only if our result is OCCT-VALID, ONE solid, and (unless
# OCCT is itself unsound on that config) volume-matches OCCT.
#
# Usage: validate_oracle.sh <exe> "<extra SESSION_ flags>" "<cfg cfg ...>" <op:cut|common|fuse>
set -u
EXE="${1:-build/Release/main_7.exe}"
FLAGS="${2:-}"
CFGS="${3:-z15 z30 z45 z90 x20 y30 z30x20 z37 x13y29 z63}"
OP="${4:-cut}"
cd "C:/pc/3_code/code_rust/session/session_cpp"
CH=serialization/boolean_steps/chairs
SP="../validation/step_probe/build/Release/step_probe.exe"
VOLTOL=0.01            # 1% relative volume tolerance vs OCCT
# "Unsound/degenerate" is DETECTED, never named: a config is oracle-untrusted iff OCCT's OWN boolean
# is not valid (OP_VALID=0, e.g. z15 common = empty) or near-zero volume (grazing, e.g. x20 common
# = 5e-4). The truth table proved the earlier hardcoded x20/z15 assumption WRONG: x20 CUT is a valid
# single OCCT solid (vol 80.30). The correct solid count is OCCT's OP_SOLIDS (z15/z30/z45 cut = 2!),
# never a hardcoded 1.

occt_flag() { case "$1" in cut) echo "--cut";; common) echo "--common";; fuse) echo "--fuse";; esac; }
field() { grep -oE "^$1 [-0-9.]+" | awk '{print $2}'; }

# Phase 1: run all configs' booleans in PARALLEL (each exports res_<cfg>_<op>.step + B_<cfg>.step).
for c in $CFGS; do
  # NOTE: no internal-volume flag -- step_probe measures OUR result's volume via OCCT independently,
  # so we only need the boolean to COMPLETE and export res_<cfg>_<op>.step.
  ( env $FLAGS SESSION_CHAIRS=$CH SESSION_CHAIRS_ROT=1 SESSION_ROT_ONLY=$c SESSION_OP=$OP \
        SESSION_ROT_STEP=1 SESSION_FAST=1 "$EXE" zzzz >/dev/null 2>&1 ) &
done
wait
# Phase 2: OCCT-gate each result.
printf "%-8s %-6s | %-26s | %-24s | %s\n" cfg op "OURS(step_probe)" "OCCT ref" "VERDICT"
printf -- "---------------------------------------------------------------------------------------------\n"
npass=0; ntot=0
for c in $CFGS; do
  ntot=$((ntot+1))
  RES="$CH/rot/res_${c}_${OP}.step"
  BROT="$CH/rot/B_${c}.step"
  if [ ! -f "$RES" ]; then printf "%-8s %-6s | %-26s\n" "$c" "$OP" "NO-RESULT-STEP (open/empty)"; continue; fi
  # 2) OCCT BRepCheck + volume of OUR result
  OURS=$("$SP" "$RES" 2>/dev/null)
  ov=$(echo "$OURS" | field VOLUME); os=$(echo "$OURS" | field SOLIDS)
  osh=$(echo "$OURS" | field SHELLS); ovalid=$(echo "$OURS" | field VALID)
  # 3) OCCT's OWN boolean truth
  OF=$(occt_flag "$OP")
  REF=$("$SP" $OF "$CH/chair0.stp" "$BROT" 2>/dev/null)
  rv=$(echo "$REF" | grep -oE "OP_VOLUME [-0-9.]+" | awk '{print $2}')
  rvalid=$(echo "$REF" | grep -oE "OP_VALID [0-9]+" | awk '{print $2}')
  rs=$(echo "$REF" | grep -oE "OP_SOLIDS [0-9]+" | awk '{print $2}')
  # 4) verdict. DEGENERATE (untrusted oracle) is DETECTED from OCCT's own output: OP not valid, or
  #    |OP_VOLUME| < 0.01 (empty/grazing). Otherwise gate: our result OCCT-VALID, solid count == OCCT's
  #    OP_SOLIDS (z15/z30/z45 cut = 2!), volume within tol of OCCT.
  degen=0
  [ "${rvalid:-0}" = "1" ] || degen=1
  awk -v v="${rv:-0}" 'BEGIN{exit (((v<0)?-v:v) < 0.01)?0:1}' && degen=1
  reason=""
  if [ "$degen" = "1" ]; then
    reason="[OCCT-degenerate: valid=${rvalid:-?} vol=${rv:-?}; not gated] "
  else
    [ "${ovalid:-0}" = "1" ] || reason="${reason}not-OCCT-VALID; "
    [ "${os:-0}" = "${rs:-1}" ] || reason="${reason}solids=${os:-?}(OCCT=${rs:-?}); "
    rel=$(awk -v a="${ov:-0}" -v b="$rv" 'BEGIN{d=a-b; if(d<0)d=-d; print (b!=0)? d/((b<0)?-b:b) : 999}')
    ok=$(awk -v r="$rel" -v t="$VOLTOL" 'BEGIN{print (r<=t)?1:0}')
    [ "$ok" = "1" ] || reason="${reason}vol ${ov} vs OCCT ${rv} (rel ${rel}); "
  fi
  if [ -z "$reason" ]; then verdict="PASS"; npass=$((npass+1)); else verdict="FAIL: $reason"; fi
  printf "%-8s %-6s | valid=%s solids=%s shells=%s vol=%s | valid=%s solids=%s vol=%s | %s\n" \
    "$c" "$OP" "${ovalid:-?}" "${os:-?}" "${osh:-?}" "${ov:-?}" "${rvalid:-?}" "${rs:-?}" "${rv:-?}" "$verdict"
done
printf -- "---------------------------------------------------------------------------------------------\n"
printf "ORACLE-VALID: %d/%d\n" "$npass" "$ntot"
