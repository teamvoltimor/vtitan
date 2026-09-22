#!/usr/bin/env bash
# The HANDOFF timing, which the 2026-09-15 afternoon rounds point at.
#
# Over the 12 Obstacles rounds of that session: 50 passes, 0 routing errors, 11
# execution failures. The discriminator is the CROSSING -- 91% of the failures
# were already on the wrong side of the pillar at commit against 23% of the
# passes that worked, crossings end legal 47% against 97% -- and the failures
# commit at p50 0.36 m against 0.74 m. Counting what blocks the router over
# 6,778 ticks with a published sign already inside activation_dist: "still holds
# an earlier sign" is 20.4% and the corner gate 9.2%; "reachable but declined"
# is ZERO. So the router is not declining, it is waiting.
#
# `pair_handoff_span_m` is the knob that governs that wait and it is ON at 0.30,
# but its blend may not START until the REAR of the chassis clears the committed
# pillar. With 0.50 m pillar spacing the blend therefore opens with almost no
# runway left, which is the same arithmetic as the late commit. A SHORTER span
# crosses harder once it opens; a LONGER one opens no earlier but finishes later;
# OFF is the control that prices the mechanism as it ships.
#
# Config only, no code. The corpus is a DAMAGE ruler here as usual. Baseline runs
# IN THIS BATCH and failure SETS are diffed, not just counts.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

SR=src/config/navigation/signs/sign_router.toml
OUT=/tmp/an/handoff
mkdir -p "$OUT"
cp "$SR" "$OUT/sign_router.toml.orig"
restore() { cp "$OUT/sign_router.toml.orig" "$ROOT/$SR"; }
trap restore EXIT

# tag|file:key=value[,file:key=value]
ARMS=(
  "baseline|"
  "span_015|$SR:pair_handoff_span_m=0.15"
  "span_050|$SR:pair_handoff_span_m=0.50"
  "span_off|$SR:pair_handoff_span_m=0.0"
)

for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG KVS <<< "$arm"
  restore
  if [ -n "$KVS" ]; then
    IFS=',' read -ra PAIRS <<< "$KVS"
    for fkv in "${PAIRS[@]}"; do
      FILE="${fkv%%:*}"; KV="${fkv#*:}"
      KEY="${KV%%=*}"; VAL="${KV#*=}"
      if ! grep -q "^${KEY} = " "$FILE"; then echo "ABORT: ${KEY} not in ${FILE}"; exit 1; fi
      sed -i "s/^${KEY} = .*/${KEY} = ${VAL}/" "$FILE"
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
  printf '%-16s failures=%-4s fixed=%-4s broke=%s\n' "$TAG" "${N:-0}" "$FIXED" "$BROKE"
done
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _ <<< "$arm"
  [ "$TAG" = "baseline" ] && continue
  F=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null); B=$(comm -13 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null)
  [ -n "$F" ] && { echo "--- $TAG FIXED:"; echo "$F" | sed 's/tests.*:://'; }
  [ -n "$B" ] && { echo "--- $TAG BROKE:"; echo "$B" | sed 's/tests.*:://'; }
done
