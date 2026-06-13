package simconfig

import "fmt"

// Section represents one of the four navigable corridors.
type Section string

const (
	SectionNorth Section = "north"
	SectionSouth Section = "south"
	SectionEast  Section = "east"
	SectionWest  Section = "west"
)

// AllSections is the canonical ordered set of all four corridors.
var AllSections = [4]Section{SectionNorth, SectionSouth, SectionEast, SectionWest}

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

// Direction represents the robot's traversal direction around the track.
type Direction string

const (
	DirectionClockwise        Direction = "clockwise"
	DirectionCounterClockwise Direction = "counterclockwise"
)

// AllDirections is the set of valid traversal directions.
var AllDirections = [2]Direction{DirectionClockwise, DirectionCounterClockwise}

// ParseDirection converts a string to a Direction enum, validating against the two valid traversal modes.
func ParseDirection(v string) (Direction, error) {
	switch Direction(v) {
	case DirectionClockwise, DirectionCounterClockwise:
		return Direction(v), nil
	}
	return "", fmt.Errorf("invalid direction %q: expected clockwise|counterclockwise", v)
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

// AllLightingScenarios is the set of valid lighting presets.
var AllLightingScenarios = [6]LightingScenario{
	LightingDirectSunlight, LightingCloudy, LightingIndoorBright,
	LightingIndoorDim, LightingEvening, LightingMixed,
}
