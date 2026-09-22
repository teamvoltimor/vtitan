#!/usr/bin/env bash
# Re-measure the REFUTED knobs against a simulator that can finally be wrong.
#
# Every one of these was rejected against an emulator with perfect colour, zero
# bearing scatter and a constant high confidence. A mechanism whose BENEFIT is
# recovery from a perception error could only ever COST there -- its upside was
# structurally unmeasurable. The baseline is now 28 with those three errors
# modelled from bags.
#
# The baseline arm runs IN THIS BATCH, not quoted from earlier: results are not
# comparable across time, and another session is editing comments in this tree.
#
# Failure SETS are compared, not just counts. Measured on this corpus earlier
# today, max_steering_rate 1.2 -> 1.8 scored 12 and 12 while FIXING one scenario
# and BREAKING another.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

E=src/config/navigation/escape/escape.toml
D=src/config/navigation/signs/sign_discovery.toml
P=src/config/navigation/motion/pursuit.toml
S=src/config/navigation/simulation/simulation.toml
OUT=/tmp/an/refuted
mkdir -p "$OUT"
for f in "$E" "$D" "$P" "$S"; do cp "$f" "$OUT/$(basename $f).orig"; done
# Absolute paths: the trap can fire from inside src/python, where every
# relative path here resolves to nothing and the tree is left edited.
restore() { for f in "$E" "$D" "$P" "$S"; do cp "$OUT/$(basename $f).orig" "$ROOT/$f"; done; }
trap restore EXIT

# tag|file|key|value   -- baseline first, then one knob at a time
ARMS=(
  "baseline|$E|tick_router_during_maneuver|false"
  "side_corr_committed|$E|side_correction_follows_committed_sign|true"
  "escape_side_committed|$E|escape_side_follows_committed_sign|true"
  "router_during_man|$E|tick_router_during_maneuver|true"
  "colour_pool_035|$D|colour_pool_radius_m|0.35"
  "min_hits_2|$D|min_hits|2"
  "steer_rate_18|$P|max_steering_rate|1.8"
  "obstacles_pushed|$S|obstacles_are_pushed|true"
  "escape_retires|$E|escape_retires_committed_sign|true"
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
for grp in (t.escape, t.sign_discovery, t.pursuit, t.simulation):
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
echo "Anything with fixed>0 is worth a second look even if the net is a wash:"
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _F _K _V <<< "$arm"
  [ "$TAG" = "baseline" ] && continue
  F=$(comm -23 "$OUT/baseline.txt" "$OUT/$TAG.txt" 2>/dev/null)
  [ -n "$F" ] && { echo "--- $TAG FIXED:"; echo "$F" | sed 's/tests.*:://'; }
done
