package core

// ScenarioType is the WRO 2026 challenge type -- mirrors
// shared.domain.enums.ScenarioType, re-exported unchanged by types.py
// because the challenge-mode jumper reuses the existing open-vs-obstacles
// concept rather than introducing a duplicate enum (see types.py's module
// docstring). Kept here rather than in a shared cross-package domain
// module: nothing else in platform/robot-go imports it yet, and
// go-architect §16 treats a shared root domain module as a deliberate
// choice for types genuinely shared across bounded contexts, not a
// default -- promote it if/when the nav-stack port needs the same type.
type ScenarioType int

const (
	// ScenarioOpen is the Open Challenge: no obstacles, lap-counting only.
	ScenarioOpen ScenarioType = iota
	// ScenarioObstacles is the Obstacle Challenge: traffic-sign pass-side
	// routing in addition to lap-counting.
	ScenarioObstacles
)

// String returns the same lowercase spelling as ScenarioType.value in the
// Python enum (e.g. "open"), matching the wire/log convention the other
// enums in this package follow.
func (s ScenarioType) String() string {
	switch s {
	case ScenarioOpen:
		return "open"
	case ScenarioObstacles:
		return "obstacles"
	default:
		return unknownEnumLabel
	}
}
