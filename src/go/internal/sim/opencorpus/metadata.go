package opencorpus

import (
	"encoding/json"
	"fmt"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// challengeTypeOpen is generate.Metadata.ChallengeType for an Open round,
// matching shared.domain.enums.ScenarioType.OPEN's wire value.
const challengeTypeOpen = "open"

// Metadata builds the scenario metadata for one parameter set, in the same
// schema cmd/simgen writes and internal/sim/scenario reads. Ports
// scenario_builder.build_open_metadata.
//
// The spawn is the starting CELL's pose, not the corridor centreline:
// StartPose supplies the yaw (which is travel-aligned and so independent of
// where across the corridor the robot sits) while the cell supplies x and y.
// Python's build_open_metadata can also spawn on the centreline, by passing
// start_cell=None -- but the centreline is not one of the legal cells, so no
// case in the enumerated space uses it and this signature does not offer it.
func Metadata(p Params, cfg startconditions.Config) (generate.Metadata, error) {
	section, ok := p.Section.Domain()
	if !ok {
		return generate.Metadata{}, fmt.Errorf("opencorpus: unknown section %q", p.Section)
	}

	widthsByName := p.Widths.MetresByName()
	widthsM := make(map[trackmodel.Section]float64, len(widthsByName))
	for name, metres := range widthsByName {
		domainSection, ok := name.Domain()
		if !ok {
			return generate.Metadata{}, fmt.Errorf("opencorpus: unknown section %q", name)
		}
		widthsM[domainSection] = metres
	}

	_, _, yaw, ok := startconditions.StartPose(section, p.Direction, widthsM, cfg, nil)
	if !ok {
		return generate.Metadata{}, fmt.Errorf(
			"opencorpus: no start pose for section %q direction %q", p.Section, p.Direction)
	}

	cells := generate.StartCells(p.Section, widthsByName[p.Section])
	if len(cells) == 0 {
		return generate.Metadata{}, fmt.Errorf("opencorpus: no start cells for section %q", p.Section)
	}
	// Wrap rather than reject, matching build_open_metadata's own
	// "cells[start_cell % len(cells)]" -- an out-of-range cell is a caller
	// error but not one worth failing a 640-scenario materialization over.
	spawn := cells[p.StartCell%len(cells)].Spawn

	widthMeta := make(map[string]generate.WidthMeta, len(SectionOrder))
	for _, name := range SectionOrder {
		widthType := simconfig.WidthTypeNarrow
		if p.Widths.IsWide(name) {
			widthType = simconfig.WidthTypeWide
		}
		widthMeta[string(name)] = generate.WidthMeta{
			Type:    widthType,
			WidthMM: p.Widths.WidthMMFor(name),
		}
	}

	return generate.Metadata{
		ScenarioID:     p.Index,
		ChallengeType:  challengeTypeOpen,
		CorridorWidths: widthMeta,
		// An Open round has no signs and no parking lot by rule. The empty
		// (not nil) slice keeps the JSON "sign_positions": [] rather than
		// null, matching what Python's model_dump emits.
		SignPositions: []generate.SignMeta{},
		NumSigns:      0,
		HasParkingLot: false,
		ParkingLot:    nil,
		StartingConditions: generate.StartingMeta{
			Direction: p.Direction.String(),
			Section:   string(p.Section),
			Position:  generate.PosMeta{X: spawn[0], Y: spawn[1]},
			Yaw:       yaw,
		},
	}, nil
}

// Write materializes params as *_metadata.json files under dir, creating it
// if needed, and returns them as a corpus ready for scenario.Orchestrator.
//
// Files rather than in-memory structs because that is the corpus contract
// every other caller already speaks: corpus.Scenario carries a path,
// scenario.NativeRunner reads and parses it, and the Python sweep tooling
// globs the same "*_metadata.json" names. Writing the space out also makes
// it inspectable -- a case that fails can be diffed or replayed on its own,
// which an in-memory-only space would not allow.
func Write(dir string, params []Params, cfg startconditions.Config) ([]corpus.Scenario, error) {
	if err := os.MkdirAll(dir, 0o750); err != nil {
		return nil, fmt.Errorf("opencorpus: creating %s: %w", dir, err)
	}

	scenarios := make([]corpus.Scenario, 0, len(params))
	for _, p := range params {
		meta, err := Metadata(p, cfg)
		if err != nil {
			return nil, err
		}
		raw, err := json.MarshalIndent(meta, "", "  ")
		if err != nil {
			return nil, fmt.Errorf("opencorpus: encoding %s: %w", p.ID(), err)
		}
		path := filepath.Join(dir, p.ID()+"_metadata.json")
		if err := os.WriteFile(path, raw, 0o600); err != nil {
			return nil, fmt.Errorf("opencorpus: writing %s: %w", path, err)
		}
		scenarios = append(scenarios, corpus.Scenario{ID: p.ID(), MetadataPath: path})
	}
	return scenarios, nil
}
