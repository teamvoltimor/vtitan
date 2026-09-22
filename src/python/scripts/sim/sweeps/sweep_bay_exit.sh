#!/usr/bin/env bash
# Make the SIMULATED in-bay start survive its own exit, which is the gate for
# measuring anything about the parking lot (adr:0086, 2026-09-17).
#
# Measured cause, 2026-09-17: with `obstacles_start_in_bay` on, the bay IS
# detected (direction_from_parking_bay answers on the first scan) and the exit
# manoeuvre DOES run -- and then the clearance guard refuses both legs from tick
# 8 onwards, returning speed 0.0 every tick while the chassis coasts the last
# 6.4 cm into a fin. Touching a fin ends the round (9.24.7), so every fixture
# dies with 0 laps. The guard's own frame is the suspect: `_dr_out` ends at
# 0.4 mm while the IMU says the chassis turned 65-73 degrees.
#
# Scored on LAPS COMPLETED with the in-bay start on, not on the corpus suite.
set -u
# Repo root derived from this script's own location (it lives at
# src/python/scripts/sim/sweeps/), so the sweep runs from any checkout on
# any machine. It used to be an absolute path to one Windows working copy.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/../../../../.." && pwd)
cd "$ROOT" || exit 1

CF=src/config/navigation/blind_nav/corridor_follower.toml
OUT=/tmp/an/bay_exit
mkdir -p "$OUT"
cp "$CF" "$OUT/corridor_follower.toml.orig"
restore() { cp "$OUT/corridor_follower.toml.orig" "$ROOT/$CF"; }
trap restore EXIT

ARMS=(
  "baseline|"
  "measured_yaw|bay_exit_dr_uses_measured_yaw=true"
  "block_ticks|bay_exit_guard_block_ticks=5"
  "tolerance|bay_exit_clearance_tolerance_m=0.01"
  "fallback|bay_exit_fallback_frames=60"
  "measured_yaw_block|bay_exit_dr_uses_measured_yaw=true,bay_exit_guard_block_ticks=5"
)

for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG KVS <<< "$arm"
  restore
  if [ -n "$KVS" ]; then
    IFS=',' read -ra PAIRS <<< "$KVS"
    for kv in "${PAIRS[@]}"; do
      KEY="${kv%%=*}"; VAL="${kv#*=}"
      if ! grep -q "^${KEY} = " "$CF"; then echo "ABORT: ${KEY} not in ${CF}"; exit 1; fi
      sed -i "s/^${KEY} = .*/${KEY} = ${VAL}/" "$CF"
    done
  fi
  echo "=== ${TAG}: ${KVS:-(baseline)}"
  cd src/python || exit 1
  VTITAN_HARDWARE_PROFILE=270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm PYTHONPATH=. \
    pixi run -e dev python scripts/sim/diag_bay_exit_start.py > "$OUT/$TAG.txt" 2>&1
  tail -1 "$OUT/$TAG.txt"
  cd "$ROOT" || exit 1
done

echo
echo "================= RESULTS ================="
for arm in "${ARMS[@]}"; do
  IFS='|' read -r TAG _ <<< "$arm"
  printf '%-20s %s\n' "$TAG" "$(tail -1 "$OUT/$TAG.txt")"
done
