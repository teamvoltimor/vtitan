#!/usr/bin/env bash
# Sweep one sim-runner flag over a list of values on a native corpus and
# print one row per value, every outcome kept separate (never one score).
#
#   scripts/transport-sweep.sh CORPUS FLAG V1,V2,... [extra sim-runner args]
#   scripts/transport-sweep.sh ../python/.corpus/obstacles/scenarios \
#       --command-delay-s 0,0.05,0.1,0.2 --command-timeout-s 0.5
#
# Meant for the transport emulation flags (--command-delay-s,
# --command-drop-rate, --command-timeout-s, --scan-delay-s), but any
# numeric native-runner flag works. Runs on the shipped TOML tree and the
# shipped hardware profiles unless SWEEP_PROFILES overrides them. The header
# records the commit and whether the tree is dirty, because a result without
# its coordinates is not comparable (adr:0087-test-methodology).
#
# Needs jq. Run from src/go.
set -euo pipefail

if [ $# -lt 3 ]; then
  sed -n '2,9p' "$0" >&2
  exit 2
fi
corpus=$1 flag=$2 values=$3
shift 3

repo_root=$(git rev-parse --show-toplevel)
profiles=${SWEEP_PROFILES:-270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm}
bin=$(mktemp -d)/sim-runner
trap 'rm -rf "$(dirname "$bin")"' EXIT
go build -o "$bin" ./cmd/sim-runner

dirty=clean
git diff --quiet HEAD || dirty=dirty
echo "# commit $(git rev-parse --short HEAD) ($dirty); corpus $corpus; profiles $profiles; sweep $flag; extra: ${*:-none}"
printf '%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\t%s\n' \
  value total succeeded laps_3 collided timed_out stuck wrong_side contact_runs mean_time_s

IFS=, read -ra arms <<<"$values"
for v in "${arms[@]}"; do
  "$bin" --runner native --corpus "$corpus" --config-root "$repo_root" \
    --hardware-profile "$profiles" --json "$flag" "$v" "$@" 2>/dev/null |
    jq -r --arg v "$v" '
      [.Results[].Result] as $r
      | [$v,
         ($r | length),
         ($r | map(select(.success)) | length),
         ($r | map(select(.laps_completed >= .target_laps)) | length),
         ($r | map(select(.collided)) | length),
         ($r | map(select(.timed_out)) | length),
         ($r | map(select(.stuck)) | length),
         ($r | map(select(.pass_side_violation)) | length),
         ($r | map(select(.contact_count > 0)) | length),
         (($r | map(.sim_time_s) | add) / ($r | length) * 10 | round / 10)]
      | @tsv'
done
