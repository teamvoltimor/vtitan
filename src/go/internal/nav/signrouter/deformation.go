// deformation.go ports
// platform/robot/src/navigation/planning/sign_router/deformation.py: the
// pass-side offset, the depth pin, and the camera-color match.

package signrouter

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// PinContext is the depth-pin tracking state ApplyDeformation/PinDepth need,
// bundled together because both are always supplied (or omitted) as a pair.
type PinContext struct {
	// RobotPos is optional (nil disables the depth pin, matching Python's
	// robot_pos=None) so callers testing the pure pass-side mapping can
	// omit it.
	RobotPos *trackmodel.Waypoint
	// YawDriftRad is the absolute heading change (rad) since the pin
	// engaged on this sign; nil matches yaw_drift=None (PIN_HEADING_GUARD
	// never releases the pin on drift alone).
	YawDriftRad *float64
}

// ApplyDeformation computes the laterally deformed waypoint for a given
// sign and corridor, matching apply_deformation. The result is clamped
// (via ClampLateral) so it can't land inside the restricted inner square or
// beyond the outer wall. Only the LATERAL coordinate carries the avoidance;
// the depth coordinate is whatever the lookahead search picked, unless
// PinDepth overrides it (see PinDepth's doc comment for why: passing depth
// through unchanged is what makes the offset arrive late).
func ApplyDeformation(
	waypoint trackmodel.Waypoint,
	sign SignSpec,
	color SignColor,
	corridor trackmodel.Section,
	direction trackmodel.Direction,
	lateralOffsetM float64,
	pin PinContext,
	cfg Config,
) trackmodel.Waypoint {
	entry, ok := routingEntry(corridor, direction)
	if !ok {
		return waypoint
	}
	mult := multForColor(entry, color)

	if entry.Axis == AxisY {
		return trackmodel.Waypoint{
			X: PinDepth(waypoint.X, sign.X, robotDepthOf(pin.RobotPos, true), pin, corridor, cfg),
			Y: ClampLateral(sign.Y+float64(mult)*lateralOffsetM, corridor, cfg),
		}
	}
	return trackmodel.Waypoint{
		X: ClampLateral(sign.X+float64(mult)*lateralOffsetM, corridor, cfg),
		Y: PinDepth(waypoint.Y, sign.Y, robotDepthOf(pin.RobotPos, false), pin, corridor, cfg),
	}
}

// robotDepthOf extracts the depth-axis coordinate from robotPos (x when
// wantX, else y), mirroring apply_deformation's `robot_pos[0] if robot_pos
// else None` / `robot_pos[1] if robot_pos else None` inline ternaries.
// Returns nil when robotPos is nil.
func robotDepthOf(robotPos *trackmodel.Waypoint, wantX bool) *float64 {
	if robotPos == nil {
		return nil
	}
	depth := robotPos.Y
	if wantX {
		depth = robotPos.X
	}
	return &depth
}

// PinDepth holds the commanded point abeam the sign instead of letting it
// recede, matching pin_depth. Applies only while the sign is genuinely
// between the chassis and the lookahead point, in whichever direction the
// robot is traveling along the corridor -- once the robot is level with
// the sign the condition lapses on its own.
//
// PinCornerGuard re-checks the robot's own real (x, y) against
// IsSquarelyInCorridor (not the raw waypoint, which the caller already
// checked upstream): the lookahead target runs 0.2-0.4m ahead of the
// robot, so the robot can already have curved out of the straight-corridor
// assumption this pin depends on while the waypoint still reads squarely
// in the corridor. PinHeadingGuard additionally releases the pin once the
// robot's heading has drifted more than PinHeadingGuardRad from where it
// stood when the pin engaged on this sign -- the position guard alone
// missed a corner-arc case where yaw rotated 67deg while position still
// read squarely in-corridor. See pin_depth's Python docstring for both
// measured regressions.
func PinDepth(
	waypointDepth, signDepth float64,
	robotDepth *float64,
	pin PinContext,
	corridor trackmodel.Section,
	cfg Config,
) float64 {
	robotPos := pin.RobotPos
	if robotDepth == nil || robotPos == nil {
		return waypointDepth
	}
	if cfg.PinCornerGuard && !IsSquarelyInCorridor(robotPos.X, robotPos.Y, corridor, cfg) {
		return waypointDepth
	}
	if cfg.PinHeadingGuard && pin.YawDriftRad != nil && *pin.YawDriftRad > cfg.PinHeadingGuardRad {
		return waypointDepth
	}
	lo, hi := *robotDepth, waypointDepth
	if lo > hi {
		lo, hi = hi, lo
	}
	if lo < signDepth && signDepth < hi {
		return signDepth
	}
	return waypointDepth
}

// MatchDetectionToSign tries to confirm a sign's color using
// world-coordinate observations, matching match_detection_to_sign.
// TrafficSignObservation already carries world coordinates, so no
// pixel-to-world projection is needed. Returns (color, true) for the
// nearest observation within cfg.DetectionMatchDistM that also clears
// cfg.MinConfidence and is RED or GREEN; (_, false) if none does.
func MatchDetectionToSign(
	observations []TrafficSignObservation,
	expectedWorldPos trackmodel.Waypoint,
	cfg Config,
) (color SignColor, ok bool) {
	bestMatchDist := math.Inf(1)
	for _, obs := range observations {
		if obs.Confidence < cfg.MinConfidence {
			continue
		}
		if obs.Color != SignColorRed && obs.Color != SignColorGreen {
			continue
		}
		world := trackmodel.Waypoint{X: obs.WorldXM, Y: obs.WorldYM}
		d := world.DistanceTo(expectedWorldPos)
		if d < cfg.DetectionMatchDistM && d < bestMatchDist {
			bestMatchDist = d
			color = obs.Color
			ok = true
		}
	}
	return color, ok
}
