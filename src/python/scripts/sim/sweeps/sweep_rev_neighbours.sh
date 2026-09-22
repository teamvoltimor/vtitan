#!/usr/bin/env bash
# Is rev_010's 13-against-25 a mechanism, or one lucky draw from a chaotic map?
#
# The parent sweep was NOT monotone -- -0.15 scored 24, -0.12 scored 29, -0.10
# scored 13, -0.08 scored 23 -- and every arm churned 8-18 fixes against 6-12
# breaks on a 25-failure set. That is the signature of scenario-level chaos. It
# also contradicts a direct measurement: at production's own contact_dist of
# 0.07 m, 94% of hardware pushes happen driving FORWARD, so rev_speed governs
# about 6% of the real exposure and cannot plausibly buy 48% of the corpus.
#
# If 0.10 is a real optimum its close neighbours land near 13. If they scatter,
# the reverse-speed axis is dead and should stop being re-tried.
#
# scoring.py accumulates `push` as the component of travel pointing AT a pillar
# while touching it, and ends the run past MAX_LEGAL_DISPLACEMENT_M (59.4 mm,
# derived from the 85 mm circle and the 50 mm pillar). So the quantity to shrink
# is TRAVEL DURING CONTACT, not contact itself.
#
# Measured 2026-09-16 (diag_bag_kturn_yaw.py, three 09-15 rounds): a latched
# K-turn travels 0.08-0.16 m and a straight OBSTACLE reverse 0.23-0.25 m, i.e.
# 1.4x to 4.2x the whole budget, open loop, because _begin_maneuver cannot react
# for its full 1.03 s.
#
# The reason this arm is not just "go slower and lose the manoeuvre": in reverse
# the chassis PIVOTS, and the measured yaw rate is FLAT in speed (1.09-1.18 rad/s
# at 0.08 m/s against 0.94-1.03 at 0.24). Cutting rev_speed should therefore cut
# travel roughly proportionally while leaving the ~48 deg per K-turn intact. If
# the corpus disagrees, the pivot model is wrong and that is worth more than the
# knob.
#
# RISK the corpus cannot see: the chassis may not start at all at 0.08 m/s under
# load. That is a hardware question (diag_bag_creep_stall.py), not a sim one.
#
# The baseline runs IN THIS BATCH. Results are not comparable across time.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1
# shellcheck source=_results.sh
source "$ROOT/src/python/scripts/sim/sweeps/_results.sh"

E=src/config/navigation/escape/escape.toml
S=src/config/navigation/simulation/simulation.toml
OUT=$ROOT/other/data/archive/sweep_rev_neighbours
mkdir -p "$OUT"
for f in "$E" "$S"; do cp "$f" "$OUT/$(basename $f).orig"; done
# Absolute paths: the trap can fire from inside src/python, where every relative
# path here resolves to nothing and the tree is left edited.
restore() { for f in "$E" "$S"; do cp "$OUT/$(basename $f).orig" "$ROOT/$f"; done; }
trap restore EXIT

# tag|file|key|value   -- baseline first, then one knob at a time
ARMS=(
  "baseline|$E|rev_speed|-0.20"
  "rev_0105|$E|rev_speed|-0.105"
  "rev_010b|$E|rev_speed|-0.10"
  "rev_0095|$E|rev_speed|-0.095"
)

for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG FILE KEY VAL <<< "$arm"
  if ! grep -q "^${KEY} = " "$FILE"; then
    echo "ABORT: ${KEY} is not in ${FILE} -- a parallel session may have rewritten it"; exit 1
  fi
  restore
  sed -i "s/^${KEY} = .*/${KEY} = ${VAL}/" "$FILE"
  cd src/python || exit 1
  unset VTITAN_HARDWARE_PROFILE
  LOADED=$(VTITAN_HARDWARE_PROFILE="270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm" \
    pixi run -e dev python -c "
from shared.config.navigation_tuning import NavigationTuning
t = NavigationTuning.load_default()
for grp in (t.escape, t.simulation):
    if hasattr(grp, '${KEY}'):
        print(getattr(grp, '${KEY}')); break
else:
    print('KEY NOT FOUND')" 2>/dev/null | tail -1)
  echo "=== ${TAG}: ${KEY}=${VAL}  loader returned: ${LOADED}"
  pixi run -e dev pytest tests/unit/test_obstacles_challenge_sim.py \
    -p no:randomly --tb=no -q -rf > "$OUT/$TAG.raw" 2>&1
  grep '^FAILED' "$OUT/$TAG.raw" | sed 's/^FAILED //' | sed 's/ - .*//' | sort > "$OUT/$TAG.txt"
  if ! grep -qE "^[0-9]+ (failed|passed)" "$OUT/$TAG.raw"; then
    echo "    ABORT: ${TAG} produced no test result -- config or import error"; tail -3 "$OUT/$TAG.raw"; exit 1
  fi
  echo "    $(tail -1 "$OUT/$TAG.raw")"
  record_arm "$(basename "$0" .sh)" "$TAG" "${KEY:+${FILE}:${KEY}=${VAL}}" "$OUT/$TAG.raw"
  cd "$ROOT" || exit 1
done

echo
echo "================= RESULTS ================="
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _F _K _V <<< "$arm"
  N=$(grep -oE '^[0-9]+ failed' "$OUT/$TAG.raw" 2>/dev/null | head -1 | cut -d' ' -f1)
  FIXED=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null | wc -l)
  BROKE=$(comm -13 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null | wc -l)
  printf '%-24s failures=%-4s fixed=%-4s broke=%s\n' "$TAG" "${N:-?}" "$FIXED" "$BROKE"
done
echo
echo "A wash in the COUNT can still be a real move in the SET:"
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _F _K _V <<< "$arm"
  [ "$TAG" = "baseline" ] && continue
  F=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null)
  [ -n "$F" ] && { echo "--- $TAG FIXED:"; echo "$F" | sed 's/tests.*:://'; }
  B=$(comm -13 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null)
  [ -n "$B" ] && { echo "--- $TAG BROKE:"; echo "$B" | sed 's/tests.*:://'; }
done
