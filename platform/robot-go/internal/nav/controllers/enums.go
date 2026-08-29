package controllers

// RiskLevel is the collision risk classification for navigation logic,
// matching shared.domain.enums.RiskLevel.
type RiskLevel int

// ThreatDirection is the LIDAR threat sector a detected obstacle lies in,
// relative to the robot, matching shared.domain.enums.ThreatDirection.
type ThreatDirection int

// ManeuverType is the kind of escape maneuver commanded by
// CollisionAvoidanceController.ComputeEscapeManeuver, matching
// shared.domain.enums.ManeuverType. StuckReverse/StuckForward are mirrored
// for completeness (the wire format other tooling reads) even though
// nothing in this package emits them yet -- they're produced by the
// not-yet-ported core navigator's stuck-escape logic, not StuckDetector
// itself.
type ManeuverType int

// unknownLabel is the default-case String() label shared by every enum in
// this file, matching each Python enum's fallback repr.
const unknownLabel = "unknown"

const (
	// RiskSafe matches RiskLevel.SAFE.
	RiskSafe RiskLevel = iota
	// RiskObstacle matches RiskLevel.OBSTACLE.
	RiskObstacle
	// RiskCritical matches RiskLevel.CRITICAL.
	RiskCritical
)

const (
	// ThreatFront matches ThreatDirection.FRONT.
	ThreatFront ThreatDirection = iota
	// ThreatBack matches ThreatDirection.BACK.
	ThreatBack
	// ThreatLeft matches ThreatDirection.LEFT.
	ThreatLeft
	// ThreatRight matches ThreatDirection.RIGHT.
	ThreatRight
	// ThreatNone matches ThreatDirection.NONE.
	ThreatNone
)

const (
	// ManeuverKTurn matches ManeuverType.K_TURN.
	ManeuverKTurn ManeuverType = iota
	// ManeuverSideCorrection matches ManeuverType.SIDE_CORRECTION.
	ManeuverSideCorrection
	// ManeuverStuckReverse matches ManeuverType.STUCK_REVERSE.
	ManeuverStuckReverse
	// ManeuverStuckForward matches ManeuverType.STUCK_FORWARD.
	ManeuverStuckForward
)

// String returns a short label for logging.
func (r RiskLevel) String() string {
	switch r {
	case RiskSafe:
		return "safe"
	case RiskObstacle:
		return "obstacle"
	case RiskCritical:
		return "critical"
	default:
		return unknownLabel
	}
}

// String returns a short label for logging.
func (t ThreatDirection) String() string {
	switch t {
	case ThreatFront:
		return "front"
	case ThreatBack:
		return "back"
	case ThreatLeft:
		return "left"
	case ThreatRight:
		return "right"
	case ThreatNone:
		return "none"
	default:
		return unknownLabel
	}
}

// String returns a short label for logging.
func (m ManeuverType) String() string {
	switch m {
	case ManeuverKTurn:
		return "k_turn"
	case ManeuverSideCorrection:
		return "side_correction"
	case ManeuverStuckReverse:
		return "stuck_reverse"
	case ManeuverStuckForward:
		return "stuck_forward"
	default:
		return unknownLabel
	}
}
