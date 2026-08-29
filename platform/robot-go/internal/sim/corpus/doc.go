// Package corpus loads the existing JSON scenario corpora used for
// sim-corpus parity testing.
//
// The Python simulator's own catalog loader
// (platform/robot/src/simulation/scenario_catalog.py, _load_fixture_scenarios)
// discovers scenarios by globbing "*_metadata.json" under a directory —
// both the 16/28-scenario committed fixture sets
// (platform/robot/tests/fixtures/scenarios/{obstacles,open}/) and the larger
// generated corpora (task gen:corpus, platform/gazebo/generator's simgen)
// follow that same layout and naming convention. This package mirrors that
// discovery rule exactly rather than inventing a second corpus format, so a
// directory built for the Python sweep tooling works unmodified as input
// here.
package corpus
