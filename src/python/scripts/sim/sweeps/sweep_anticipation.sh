#!/usr/bin/env bash
# COUPLED sweep of the anticipation budget, 3 x 2.
#
# Every A/B in this project has moved ONE knob, which is exactly what cannot
# find a coupled optimum: measured today, max_steering_rate 1.2 -> 1.8 scored
# 12 and 12 while FIXING go_obstacles_0008 and BREAKING 0004. The band-change
# deficit (0.698 m needed, 0.36 m committed) is split across perception and the
# planner, so neither side alone can close it.
#
#   sign_router.sign_lane_commit_ahead_m : 0.00 (ships, = at the sign) / 0.20 / 0.40
#   sign_discovery.min_hits              : 3 (ships) / 2
#
# CONTROL: each arm prints the value the tuning loader actually returns. A sweep
# whose arms are byte-identical has happened here before (the min-turn-radius
# axis) and reads as a clean null.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

R=src/config/navigation/signs/sign_router.toml
D=src/config/navigation/signs/sign_discovery.toml
OUT=/tmp/an/sweep
mkdir -p "$OUT"
cp "$R" "$OUT/router.toml.orig"
cp "$D" "$OUT/discovery.toml.orig"
restore() { cp "$OUT/router.toml.orig" "$R"; cp "$OUT/discovery.toml.orig" "$D"; }
trap restore EXIT

for AHEAD in 0.00 0.20 0.40; do
  for HITS in 3 2; do
    TAG="ahead${AHEAD}_hits${HITS}"
    restore
    sed -i "s/^sign_lane_commit_ahead_m = .*/sign_lane_commit_ahead_m = ${AHEAD}/" "$R"
    sed -i "s/^min_hits = .*/min_hits = ${HITS}/" "$D"

    cd src/python || exit 1
    unset VTITAN_HARDWARE_PROFILE
    # CONTROL: what did the loader actually read?
    LOADED=$(VTITAN_HARDWARE_PROFILE="270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm"       pixi run -e dev python -c "
from shared.config.navigation_tuning import NavigationTuning
t = NavigationTuning.load_default()
print(f'{t.sign_router.sign_lane_commit_ahead_m} {t.sign_discovery.min_hits}')
" 2>/dev/null | tail -1)
    echo "=== ${TAG}  loader returned: ${LOADED} (wanted ${AHEAD} ${HITS})"
    pixi run -e dev pytest tests/unit/test_obstacles_challenge_sim.py \
      -p no:randomly --tb=no -q -rf > "$OUT/$TAG.raw" 2>&1
    grep '^FAILED' "$OUT/$TAG.raw" | sed 's/^FAILED //' | sed 's/ - .*//' | sort > "$OUT/$TAG.txt"
    echo "    $(tail -1 "$OUT/$TAG.raw")"
    cd "$ROOT" || exit 1
  done
done

echo
echo "================ GRID (failures, lower is better) ================"
printf '%-10s %8s %8s\n' "ahead" "hits=3" "hits=2"
for AHEAD in 0.00 0.20 0.40; do
  A=$(grep -oE '^[0-9]+ failed' "$OUT/ahead${AHEAD}_hits3.raw" 2>/dev/null | head -1 | cut -d' ' -f1)
  B=$(grep -oE '^[0-9]+ failed' "$OUT/ahead${AHEAD}_hits2.raw" 2>/dev/null | head -1 | cut -d' ' -f1)
  printf '%-10s %8s %8s\n' "$AHEAD" "${A:-?}" "${B:-?}"
done
echo
echo "Baseline cell is ahead0.00_hits3. Compare SETS, not just counts:"
for AHEAD in 0.00 0.20 0.40; do
  for HITS in 3 2; do
    T="ahead${AHEAD}_hits${HITS}"
    [ -f "$OUT/$T.txt" ] || continue
    echo "  $T : $(comm -13 "$OUT/ahead0.00_hits3.txt" "$OUT/$T.txt" | wc -l) new, $(comm -23 "$OUT/ahead0.00_hits3.txt" "$OUT/$T.txt" | wc -l) fixed"
  done
done
