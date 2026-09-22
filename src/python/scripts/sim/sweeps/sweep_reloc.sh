#!/usr/bin/env bash
# Does the long-jump confirmation guard cost the corpus anything?
#
# `relocalize_confirm_dist_m` holds a global relocalization winner further than
# 1.00 m from the prior and takes it only when a LATER global search reconverges
# to it. It exists for run_20260915_160804: believed widths 0.63/0.955/0.958, so
# the width-spread gate passed, the search ran, and the estimate teleported
# 2.06 m in 1.26 s to very nearly the mirror of the pose it left -- then 26
# waypoints of backwards replan and 213 deg of yaw in 4.8 s inside an 8 x 26 cm
# box, the U-turn on the round that scored 6.
#
# The corpus cannot show the BENEFIT: Obstacles fixes all four corridors at
# 1000 mm, so the spread gate already refuses the search there and this guard is
# downstream of it. It CAN show damage, which is the whole point of running it --
# the guard delays a genuine long rescue by relocalize_after_scans (~1.5 s), and
# if any scenario depends on a prompt one it will fail here.
#
# "baseline" is the guard ON, as shipped; "confirm_off" restores the behaviour
# before it. Failure SETS are diffed, not just counts.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

LOC=src/config/navigation/blind_nav/localization.toml
OUT=/tmp/an/reloc
mkdir -p "$OUT"
cp "$LOC" "$OUT/localization.toml.orig"
restore() { cp "$OUT/localization.toml.orig" "$ROOT/$LOC"; }
trap restore EXIT

# tag|file:key=value[,file:key=value]
ARMS=(
  "baseline|"
  "confirm_off|$LOC:relocalize_confirm_dist_m=0.0"
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
