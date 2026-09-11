package simconfig

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// Section represents one of the four navigable corridors.
type Section string

const (
	SectionNorth Section = "north"
	SectionSouth Section = "south"
	SectionEast  Section = "east"
	SectionWest  Section = "west"
)

var (
	// AllSections is the canonical ordered set of all four corridors.
	AllSections = [4]Section{SectionNorth, SectionSouth, SectionEast, SectionWest}
)

// Capitalized returns the section name with a leading capital, e.g. "North".
func (s Section) Capitalized() string {
	if len(s) == 0 {
		return ""
	}
	b := []byte(s)
	if b[0] >= 'a' && b[0] <= 'z' {
		b[0] -= 32
	}
	return string(b)
}

// ParseSection converts a string to a Section enum, validating against the four known corridors.
func ParseSection(v string) (Section, error) {
	switch Section(v) {
	case SectionNorth, SectionSouth, SectionEast, SectionWest:
		return Section(v), nil
	}
	return "", fmt.Errorf("invalid section %q: expected north|south|east|west", v)
}

// Domain returns the navigation domain's Section for this wire/sim section.
// The two enums describe the same four corridors with different
// representations (this one is the metadata/SDF wire name, trackmodel's is
// the iota-based domain type), so every cross-over goes through here
// instead of a bespoke map per package. ok is false for a section outside
// the four known corridors.
func (s Section) Domain() (trackmodel.Section, bool) {
	switch s {
	case SectionNorth:
		return trackmodel.North, true
	case SectionSouth:
		return trackmodel.South, true
	case SectionEast:
		return trackmodel.East, true
	case SectionWest:
		return trackmodel.West, true
	}
	return 0, false
}

// Direction represents the robot's traversal direction around the track.
type Direction string

const (
	DirectionClockwise        Direction = "clockwise"
	DirectionCounterClockwise Direction = "counterclockwise"
)

var (
	// AllDirections is the set of valid traversal directions.
	AllDirections = [2]Direction{DirectionClockwise, DirectionCounterClockwise}
)

// ParseDirection converts a string to a Direction enum, validating against the two valid traversal modes.
func ParseDirection(v string) (Direction, error) {
	switch Direction(v) {
	case DirectionClockwise, DirectionCounterClockwise:
		return Direction(v), nil
	}
	return "", fmt.Errorf("invalid direction %q: expected clockwise|counterclockwise", v)
}

// Domain returns the navigation domain's Direction for this wire/sim
// direction, mirroring Section.Domain. ok is false outside the two known
// values.
func (d Direction) Domain() (trackmodel.Direction, bool) {
	switch d {
	case DirectionClockwise:
		return trackmodel.Clockwise, true
	case DirectionCounterClockwise:
		return trackmodel.Counterclockwise, true
	}
	return 0, false
}

// ScenarioType identifies the WRO challenge variant.
type ScenarioType string

const (
	ScenarioTypeOpen      ScenarioType = "open"
	ScenarioTypeObstacles ScenarioType = "obstacles"
)

// ParseScenarioType converts a string to a ScenarioType enum, validating against the two WRO challenges.
func ParseScenarioType(v string) (ScenarioType, error) {
	switch ScenarioType(v) {
	case ScenarioTypeOpen, ScenarioTypeObstacles:
		return ScenarioType(v), nil
	}
	return "", fmt.Errorf("invalid scenario type %q: expected open|obstacles", v)
}

// LightingScenario identifies one of the six pre-defined lighting presets.
type LightingScenario string

const (
	LightingDirectSunlight LightingScenario = "direct_sunlight"
	LightingCloudy         LightingScenario = "cloudy"
	LightingIndoorBright   LightingScenario = "indoor_bright"
	LightingIndoorDim      LightingScenario = "indoor_dim"
	LightingEvening        LightingScenario = "evening"
	LightingMixed          LightingScenario = "mixed"
)

var (
	// AllLightingScenarios is the set of valid lighting presets.
	AllLightingScenarios = [6]LightingScenario{
		LightingDirectSunlight, LightingCloudy, LightingIndoorBright,
		LightingIndoorDim, LightingEvening, LightingMixed,
	}
)
