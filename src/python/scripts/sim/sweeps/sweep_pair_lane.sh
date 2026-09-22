#!/usr/bin/env bash
# The in-section opposite-colour PAIR (green then red, 1.0 m apart) is what the
# 2026-09-15 afternoon Obstacles rounds failed on: 6 of 8 execution failures are
# the second pillar of such a pair or the red before the lot. The lane plans two
# plateaux (hold_m either side of each sign) and a straight S-bend between them:
# with hold 0.25 that S-bend is 0.50 m long for a 0.644 m lateral shift, which
# the chassis (R capped at 0.35) cannot deliver. Two knobs shrink the demand:
# a shorter hold lengthens the S-bend, a smaller clearance margin shrinks the shift.
#
# Baseline runs IN THIS BATCH. Failure SETS are diffed, not just counts.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

SR=src/config/navigation/signs/sign_router.toml
OUT=/tmp/an/pair_lane
mkdir -p "$OUT"
cp "$SR" "$OUT/sign_router.toml.orig"
restore() { cp "$OUT/sign_router.toml.orig" "$ROOT/$SR"; }
trap restore EXIT

# tag|key=value[,key=value]
ARMS=(
  "baseline|"
  "hold_010|sign_lane_hold_m=0.10"
  "margin_005|sign_clearance_margin_m=0.05"
  "hold_010_margin_005|sign_lane_hold_m=0.10,sign_clearance_margin_m=0.05"
  "margin_000|sign_clearance_margin_m=0.0"
)

for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG KVS <<< "$arm"
  restore
  if [ -n "$KVS" ]; then
    IFS=',' read -ra PAIRS <<< "$KVS"
    for kv in "${PAIRS[@]}"; do
      KEY="${kv%%=*}"; VAL="${kv#*=}"
      if ! grep -q "^${KEY} = " "$SR"; then echo "ABORT: ${KEY} not in ${SR}"; exit 1; fi
      sed -i "s/^${KEY} = .*/${KEY} = ${VAL}/" "$SR"
    done
  fi
  echo "=== ${TAG}: ${KVS:-(baseline)}"
  cd src/python || exit 1
  unset VTITAN_HARDWARE_PROFILE
  pixi run -e dev pytest tests/unit/test_obstacles_challenge_sim.py \
    -p no:randomly --tb=no -q -rf > "$OUT/$TAG.raw" 2>&1
  grep '^FAILED' "$OUT/$TAG.raw" | sed 's/^FAILED //' | sed 's/ - .*//' | sort > "$OUT/$TAG.txt"
  if ! grep -qE "^[0-9]+ (failed|passed)" "$OUT/$TAG.raw"; then
    echo "    ABORT: ${TAG} produced no test result"; tail -3 "$OUT/$TAG.raw"; exit 1
  fi
  echo "    $(tail -1 "$OUT/$TAG.raw")"
  cd "$ROOT" || exit 1
done

echo
echo "================= RESULTS ================="
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _ <<< "$arm"
  N=$(grep -oE '^[0-9]+ failed' "$OUT/$TAG.raw" 2>/dev/null | head -1 | cut -d' ' -f1)
  FIXED=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null | wc -l)
  BROKE=$(comm -13 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null | wc -l)
  printf '%-22s failures=%-4s fixed=%-4s broke=%s\n' "$TAG" "${N:-?}" "$FIXED" "$BROKE"
done
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _ <<< "$arm"
  [ "$TAG" = "baseline" ] && continue
  F=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null); B=$(comm -13 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null)
  [ -n "$F" ] && { echo "--- $TAG FIXED:"; echo "$F" | sed 's/tests.*:://'; }
  [ -n "$B" ] && { echo "--- $TAG BROKE:"; echo "$B" | sed 's/tests.*:://'; }
done
