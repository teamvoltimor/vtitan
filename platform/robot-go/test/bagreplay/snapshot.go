package bagreplay

import (
	"encoding/json"
	"fmt"
)

// NavDebugSnapshot is one recorded /nav_debug message: the Python
// CoreNavigator's complete per-tick internal state, mirroring
// shared.domain.models.NavigatorDebugSnapshot field for field.
//
// Nullable fields are pointers for the reason that model's own docstring
// gives: None means "not computed on this tick's branch", not "unknown" and
// not zero. Collapsing that to a zero value would make a parity diff read a
// deliberately-absent crosstrack error as a measured 0.0.
//
// Fields with no Go counterpart are kept anyway (parking, blind creep). They
// are exactly the modules this port has not reached yet, so a diff needs to
// be able to SEE them in the reference in order to report them as
// unported-and-skipped rather than silently matching.
type NavDebugSnapshot struct {
	Phase string `json:"phase"`

	// Pose and race state -- available on every phase except "no_pose".
	PoseX           *float64 `json:"pose_x"`
	PoseY           *float64 `json:"pose_y"`
	PoseYaw         *float64 `json:"pose_yaw"`
	Direction       *string  `json:"direction"`
	CurrentCorridor *string  `json:"current_corridor"`
	WaypointIndex   *int     `json:"waypoint_index"`
	LapsCompleted   int      `json:"laps_completed"`
	NumLaps         int      `json:"num_laps"`

	// Stuck detection -- set whenever the detector runs.
	IsStuck         *bool    `json:"is_stuck"`
	StuckCount      *int     `json:"stuck_count"`
	RecentMovementM *float64 `json:"recent_movement_m"`

	// Perception / risk -- set on the normal_drive phase.
	ForwardClearanceM *float64 `json:"forward_clearance_m"`
	MinLidarRangeM    *float64 `json:"min_lidar_range_m"`
	Risk              *string  `json:"risk"`
	EscapeRisk        *string  `json:"escape_risk"`
	RearClearanceM    *float64 `json:"rear_clearance_m"`

	// Path tracking -- set on the normal_drive phase.
	CrosstrackErrorM   *float64 `json:"crosstrack_error_m"`
	LookaheadDistanceM *float64 `json:"lookahead_distance_m"`
	PathTurnAheadRad   *float64 `json:"path_turn_ahead_rad"`
	SteerTargetX       *float64 `json:"steer_target_x"`
	SteerTargetY       *float64 `json:"steer_target_y"`
	AngleErrorRad      *float64 `json:"angle_error_rad"`

	// Speed selection -- set on the normal_drive phase.
	ClearanceSpeedMPS *float64 `json:"clearance_speed_mps"`
	HeadingSpeedMPS   *float64 `json:"heading_speed_mps"`

	// Final command -- set on every phase that publishes a drive command.
	CommandedSpeedMPS     *float64 `json:"commanded_speed_mps"`
	CommandedSteeringNorm *float64 `json:"commanded_steering_norm"`

	// Escape/stuck maneuver -- set whenever one is latched or begun.
	ActiveManeuverType *string  `json:"active_maneuver_type"`
	ManeuverSteering   *float64 `json:"maneuver_steering"`
	ManeuverSpeedMPS   *float64 `json:"maneuver_speed_mps"`
	ManeuverFramesLeft *int     `json:"maneuver_frames_left"`
	EscapeCount        *int     `json:"escape_count"`

	// Parking -- no Go counterpart yet (no ParkController in the port).
	ParkingEngaged *bool   `json:"parking_engaged"`
	ParkPhase      *string `json:"park_phase"`

	// Obstacles sign routing -- set whenever a SignRouter is attached.
	ActiveSignCount      *int     `json:"active_sign_count"`
	SignDeformMagnitudeM *float64 `json:"sign_deform_magnitude_m"`

	// Blind creep / direction inference -- no Go counterpart yet.
	DirectionGateVerdict           *string  `json:"direction_gate_verdict"`
	DirectionLeftRangeM            *float64 `json:"direction_left_range_m"`
	DirectionRightRangeM           *float64 `json:"direction_right_range_m"`
	DirectionVotesClockwise        *int     `json:"direction_votes_clockwise"`
	DirectionVotesCounterclockwise *int     `json:"direction_votes_counterclockwise"`
	CorridorWidthBeliefM           *float64 `json:"corridor_width_belief_m"`
}

// DecodeNavDebug decodes one recorded /nav_debug message, matching
// bag_io.decode_nav_debug: unwrap the std_msgs/String envelope, then parse
// the pydantic JSON inside it.
func DecodeNavDebug(data []byte) (NavDebugSnapshot, error) {
	payload, err := DecodeStdMsgsString(data)
	if err != nil {
		return NavDebugSnapshot{}, err
	}
	var snapshot NavDebugSnapshot
	if unmarshalErr := json.Unmarshal([]byte(payload), &snapshot); unmarshalErr != nil {
		return NavDebugSnapshot{}, fmt.Errorf("bagreplay: parsing /nav_debug JSON: %w", unmarshalErr)
	}
	return snapshot, nil
}
