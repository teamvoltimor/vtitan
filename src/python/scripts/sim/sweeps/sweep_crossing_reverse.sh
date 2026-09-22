#!/usr/bin/env bash
# Operator's design 2026-09-17: when a committed crossing's arc does not fit the
# run-up left, back straight BEFORE any contact so the crossing has more room.
# Budget of k_turn_min_s legs per committed sign; fit judged at the slow tier.
#
# Baseline runs IN THIS BATCH. Failure SETS are diffed, not just counts.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

SR=src/config/navigation/signs/sign_router.toml
OUT=/tmp/an/crossing_reverse
mkdir -p "$OUT"
cp "$SR" "$OUT/sign_router.toml.orig"
restore() { cp "$OUT/sign_router.toml.orig" "$ROOT/$SR"; }
trap restore EXIT

# tag|key=value[,key=value]
ARMS=(
  "baseline|"
  "legs1|sign_crossing_reverse_legs=1"
  "legs2|sign_crossing_reverse_legs=2"
  "legs3|sign_crossing_reverse_legs=3"
  "legs2_fit022|sign_crossing_reverse_legs=2,sign_crossing_reverse_fit_mps=0.22"
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
