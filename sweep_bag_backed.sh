#!/usr/bin/env bash
# The two knobs that ship OFF for a SIM reason while the BAGS argue for them:
#
# * `tick_router_during_maneuver` -- off and UNVALIDATED, never refuted. Over 105
#   escape episodes the escape gains a median 9.8 cm and 97% of them are handed
#   back the SAME target (median movement 0 cm), 62% re-firing within 2 s; 183
#   manoeuvre episodes cover 22.3% of ticks with the plan frozen for up to 44
#   ticks. It replays the INGEST half only and drops the deformed waypoint, so it
#   cannot move the wheel -- it buys a fresh map on the far side of the manoeuvre.
# * `sign_lane_deform_fallback_m` -- over 129 bags, on FAILED crossing passes the
#   lane was on the legal side 37.4% of the time against the deform's 47.3%
#   (chi2 17.0, p=4e-5). It ships off because the simulator's sign map is EXACT,
#   so the lane always materialises and the branch barely fires: a flat sim A/B
#   measures the sim, not the knob.
#
# So the corpus here is a DAMAGE ruler, not a benefit one. Baseline runs IN THIS
# BATCH. Failure SETS are diffed, not just counts.
set -u
ROOT=/d/Dev/active/projects/teamvoltimor/vtitan
cd "$ROOT" || exit 1

ESC=src/config/navigation/escape/escape.toml
SR=src/config/navigation/signs/sign_router.toml
OUT=/tmp/an/bag_backed
mkdir -p "$OUT"
cp "$ESC" "$OUT/escape.toml.orig"
cp "$SR" "$OUT/sign_router.toml.orig"
restore() { cp "$OUT/escape.toml.orig" "$ROOT/$ESC"; cp "$OUT/sign_router.toml.orig" "$ROOT/$SR"; }
trap restore EXIT

# tag|file:key=value[,file:key=value]
ARMS=(
  "baseline|"
  "tick_router|$ESC:tick_router_during_maneuver=true"
  "fallback_013|$SR:sign_lane_deform_fallback_m=0.13"
  "fallback_028|$SR:sign_lane_deform_fallback_m=0.28"
  "both|$ESC:tick_router_during_maneuver=true,$SR:sign_lane_deform_fallback_m=0.13"
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
