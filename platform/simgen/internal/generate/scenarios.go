// Package generate implements WRO 2026 scenario randomization.
package generate

import (
	"fmt"

	"voldemorbot/simgen/internal/simconfig"
)

// scenarioEntry is one pillar in the South-corridor template frame.
// x = depth (1.0 near, 1.5 mid, 2.0 far), y = width (0.4 outer, 0.6 inner).
type scenarioEntry struct {
	color string
	x, y  float64
}

// scenarios is the WRO 2026 official 36-scenario traffic-sign table.
// All entries are defined in South-corridor frame; use ApplyScenarioToSection
// to transform them to world coordinates for any corridor.
//
// Scenarios 1–12: single pillar.
// Scenarios 13–36: double pillar.
var scenarios = map[int][]scenarioEntry{
	// Single pillar — inner position (y=0.6)
	1: {{simconfig.ColorNameGreen, 1.0, 0.6}},
	2: {{simconfig.ColorNameRed, 1.0, 0.6}},
	3: {{simconfig.ColorNameGreen, 1.5, 0.6}},
	4: {{simconfig.ColorNameRed, 1.5, 0.6}},
	5: {{simconfig.ColorNameGreen, 2.0, 0.6}},
	6: {{simconfig.ColorNameRed, 2.0, 0.6}},
	// Single pillar — outer position (y=0.4)
	7:  {{simconfig.ColorNameGreen, 1.0, 0.4}},
	8:  {{simconfig.ColorNameRed, 1.0, 0.4}},
	9:  {{simconfig.ColorNameGreen, 1.5, 0.4}},
	10: {{simconfig.ColorNameRed, 1.5, 0.4}},
	11: {{simconfig.ColorNameGreen, 2.0, 0.4}},
	12: {{simconfig.ColorNameRed, 2.0, 0.4}},
	// Double pillar
	13: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	14: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.6}},
	15: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	16: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.6}},
	17: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	18: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.6}},
	19: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	20: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.4}},
	21: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	22: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.4}},
	23: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	24: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.4}},
	25: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	26: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.6}},
	27: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	28: {{simconfig.ColorNameGreen, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.6}},
	29: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameGreen, 2.0, 0.6}},
	30: {{simconfig.ColorNameRed, 1.0, 0.6}, {simconfig.ColorNameRed, 2.0, 0.6}},
	31: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	32: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.4}},
	33: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	34: {{simconfig.ColorNameGreen, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.4}},
	35: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameGreen, 2.0, 0.4}},
	36: {{simconfig.ColorNameRed, 1.0, 0.4}, {simconfig.ColorNameRed, 2.0, 0.4}},
}

// ScenarioPillar is a traffic sign pillar with world coordinates and color.
type ScenarioPillar struct {
	Color string
	X, Y  float64
}

// ApplyScenarioToSection transforms the South-frame scenario coordinates to
// world coordinates for the given corridor section.
//
// The SCENARIOS table defines pillar positions in the South corridor:
//   - x is the depth axis (along the corridor, 1.0–2.0 m)
//   - y is the width axis (across the corridor, 0.4 or 0.6 m from outer wall)
//
// The transform mirrors/rotates to produce the correct world (x, y) for each
// of the four corridors. scenarioID must be in [1, 36].
func ApplyScenarioToSection(scenarioID int, section simconfig.Section) ([]ScenarioPillar, error) {
	if scenarioID < simconfig.ScenarioIDMin || scenarioID > simconfig.ScenarioIDMax {
		return nil, fmt.Errorf(
			"scenario_id must be %d–%d, got %d",
			simconfig.ScenarioIDMin,
			simconfig.ScenarioIDMax,
			scenarioID,
		)
	}
	entries := scenarios[scenarioID]
	out := make([]ScenarioPillar, len(entries))

	const trackMax = simconfig.TrackMaxCoord
	for i, e := range entries {
		var wx, wy float64
		switch section {
		case simconfig.SectionSouth:
			wx, wy = e.x, e.y
		case simconfig.SectionNorth:
			wx, wy = e.x, trackMax-e.y
		case simconfig.SectionEast:
			wx, wy = trackMax-e.y, e.x
		case simconfig.SectionWest:
			wx, wy = e.y, e.x
		default:
			return nil, fmt.Errorf("unknown section %q", section)
		}
		out[i] = ScenarioPillar{Color: e.color, X: wx, Y: wy}
	}
	return out, nil
}
