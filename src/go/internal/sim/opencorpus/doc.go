// Package opencorpus enumerates the Open Challenge scenario space and
// materializes it as scenario metadata files.
//
// Ports platform/robot/src/simulation/scenario_builder.py's
// build_open_metadata/start_cells together with the enumeration order in
// scenario_catalog.py's OpenChallengeScenarioSpace.
//
// # Why this exists
//
// Every other corpus in this repo is a directory of committed
// *_metadata.json files that internal/sim/corpus globs off disk. The Open
// Challenge is not: Python commits no Open fixtures and builds the space
// on demand, because the space is small, exhaustive and fully determined
// by the rules -- 2 directions x 16 width layouts x 4 sections x every
// legal starting cell in each = 640 scenarios. Without this package the Go
// simulator has no Open scenarios at all, which is why the width belief
// (internal/nav/widthbelief), whose entire measured payoff is an Open
// result, could be ported but not measured.
//
// cmd/simgen can already emit Open scenarios, but it RANDOMIZES them from
// a seed. That is the wrong tool here: a randomized draw cannot reproduce
// "case 300" or the cluster {38, 64, 71, 207, 264, 368, 407, 426} that
// several Open findings are recorded against. This package reproduces
// Python's enumeration exactly so a case index means the same scenario in
// both languages.
//
// # Enumeration order is load-bearing
//
// The index IS the scenario's identity across every recorded Open result,
// so Space's loop nesting mirrors OpenChallengeScenarioSpace.__post_init__
// exactly: direction, then the 4-bit width layout, then section, then
// cell. Note SectionOrder is south/north/east/west -- Python's
// _OPEN_SECTIONS -- and deliberately NOT simconfig.AllSections, which is
// north/south/east/west. Swapping those two orders silently renumbers the
// entire corpus while still producing 640 valid scenarios.
package opencorpus
