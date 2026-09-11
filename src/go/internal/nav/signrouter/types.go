package signrouter

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"

// Axis is which world coordinate a sign-routing table entry deforms,
// matching shared.domain.enums.Axis.
type Axis int

// SignColor is the detected object color/class from the vision pipeline,
// matching shared.domain.models.SignColor. RED/GREEN are the only colors
// SignSpec/TrafficSignObservation ever carry through this package; MAGENTA
// (the parking-block class the same detector emits) is kept only for
// fidelity with the Python enum's third member -- see SignColor's Python
// doc comment.
type SignColor int

// SignSpec is a routed sign's world position and color, matching
// src.navigation.planning.sign_discovery.SignSpec (a 3-field dataclass;
// sign_discovery.py itself is out of scope for this port, but the type it
// declares is the one SignRouter and sign_lane consume).
type SignSpec struct {
	X, Y  float64
	Color SignColor
}

// TrafficSignObservation is a single traffic-sign detection with world
// pose and confidence, matching shared.domain.models.TrafficSignObservation.
// Only WorldXM/WorldYM/Color/Confidence are read by anything in this
// package (match_detection_to_sign); the remaining fields are kept for
// fidelity with the Python dataclass's full shape.
type TrafficSignObservation struct {
	WorldXM             float64
	WorldYM             float64
	Color               SignColor
	Confidence          float64
	DetectedAtTimestamp float64
	InRobotFrame        bool
	// BBoxYMin/BBoxXMin/BBoxYMax/BBoxXMax mirror the Python fields' `int |
	// None` optionality with nil pointers; unused by this package.
	BBoxYMin *int
	BBoxXMin *int
	BBoxYMax *int
	BBoxXMax *int
}

// RoutingEntry is the sign-routing parameters for a (corridor, direction)
// pair, matching shared.domain.models.RoutingEntry.
type RoutingEntry struct {
	Axis      Axis
	RedMult   int
	GreenMult int
}

// LaneSpec pairs a routed sign with the corridor label the router uses for
// it, matching the tuple element type of SignRouter.lane_specs and
// sign_lane.apply_sign_lanes' signs parameter.
type LaneSpec struct {
	Spec     SignSpec
	Corridor trackmodel.Section
}

// RoutedSign pairs a routed sign's world position with its own corridor,
// matching SignRouter.routed_sign_positions_by_corridor's element type.
type RoutedSign struct {
	Waypoint trackmodel.Waypoint
	Corridor trackmodel.Section
}

const (
	// AxisX matches Axis.X: the routing table deforms the x-coordinate.
	AxisX Axis = iota
	// AxisY matches Axis.Y: the routing table deforms the y-coordinate.
	AxisY
)

const (
	// SignColorRed matches SignColor.RED.
	SignColorRed SignColor = iota
	// SignColorGreen matches SignColor.GREEN.
	SignColorGreen
	// SignColorMagenta matches SignColor.MAGENTA -- never produced or
	// consumed by anything in this package, kept only so SignColor mirrors
	// the domain enum's full value set.
	SignColorMagenta
)
