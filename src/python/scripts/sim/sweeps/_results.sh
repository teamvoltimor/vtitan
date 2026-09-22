# Sourced by the sweeps in this directory. Records each finished arm in the
# results store (src/tools/sweep_results.py) with its commit, working-tree
# diff, corpus and profile, so a failure set outlives /tmp and the terminal.
# See adr:0087-test-methodology for why those coordinates are required.
#
# Recording is best-effort: a store failure prints a note and never fails the
# sweep, whose own output stays the primary result.
#
# record_arm SWEEP ARM OVERRIDES RAW_PYTEST_OUTPUT
record_arm() {
  local out
  if out=$(python3 "$ROOT/src/tools/sweep_results.py" record \
      --sweep "$1" --arm "$2" --overrides "$3" \
      --corpus tests/unit/test_obstacles_challenge_sim.py --raw "$4" 2>&1); then
    echo "    $out"
  else
    echo "    (not recorded in the results store: $out)"
  fi
}
