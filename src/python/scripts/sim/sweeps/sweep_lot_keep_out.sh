#!/usr/bin/env bash
# Narrow lot keep-out 2026-09-17: hold the pursuit target off the parking fins
# while driving PAST them, only in the lot corridor and only when no committed
# sign wants that side. The general keep-out is already refuted at 12 -> 30.
#
# Baseline runs IN THIS BATCH. Failure SETS are diffed, not just counts.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

SR=src/config/navigation/parking/parking.toml
OUT=/tmp/an/lot_keep_out
mkdir -p "$OUT"
cp "$SR" "$OUT/parking.toml.orig"
restore() { cp "$OUT/parking.toml.orig" "$ROOT/$SR"; }
trap restore EXIT

# tag|key=value[,key=value]
ARMS=(
  "baseline|"
  "keep_002|lot_keep_out_m=0.02"
  "keep_005|lot_keep_out_m=0.05"
  "keep_010|lot_keep_out_m=0.10"
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
