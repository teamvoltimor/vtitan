package navigator

import "encoding/json"

// wireSnapshot is DebugSnapshot in the shape CoreNavigator publishes on
// /nav_debug: shared.domain.models.NavigatorDebugSnapshot's
// model_dump_json(), a flat snake_case object with null for anything the
// current phase did not compute.
//
// Written out explicitly rather than derived from the protobuf. The proto
// nests the pose as x/y/yaw inside a sub-message where this format is flat
// pose_x/pose_y/pose_yaw, so protojson would emit a different document that
// happens to carry the same numbers -- and every diag_bag_*.py script keys
// off these exact names.
//
// The Go navigator does not produce every field the Python one does. The
// direction-gate, width-belief and parking-engaged fields have no Go
// counterpart yet and marshal as null, which is what a reader should see:
// "this run did not report it", not a fabricated zero.
type wireSnapshot struct {
	Phase string `json:"phase"`

	PoseX           *float64 `json:"pose_x"`
	PoseY           *float64 `json:"pose_y"`
	PoseYaw         *float64 `json:"pose_yaw"`
	Direction       *string  `json:"direction"`
	CurrentCorridor *string  `json:"current_corridor"`
	WaypointIndex   *int     `json:"waypoint_index"`
	LapsCompleted   int      `json:"laps_completed"`
	NumLaps         int      `json:"num_laps"`

	IsStuck         *bool    `json:"is_stuck"`
	StuckCount      *int     `json:"stuck_count"`
	RecentMovementM *float64 `json:"recent_movement_m"`

	ForwardClearanceM *float64 `json:"forward_clearance_m"`
	MinLidarRangeM    *float64 `json:"min_lidar_range_m"`
	Risk              *string  `json:"risk"`
	EscapeRisk        *string  `json:"escape_risk"`
	RearClearanceM    *float64 `json:"rear_clearance_m"`

	CrosstrackErrorM   *float64 `json:"crosstrack_error_m"`
	LookaheadDistanceM *float64 `json:"lookahead_distance_m"`
	PathTurnAheadRad   *float64 `json:"path_turn_ahead_rad"`
	SteerTargetX       *float64 `json:"steer_target_x"`
	SteerTargetY       *float64 `json:"steer_target_y"`
	AngleErrorRad      *float64 `json:"angle_error_rad"`

	ClearanceSpeedMPS *float64 `json:"clearance_speed_mps"`
	HeadingSpeedMPS   *float64 `json:"heading_speed_mps"`

	CommandedSpeedMPS     *float64 `json:"commanded_speed_mps"`
	CommandedSteeringNorm *float64 `json:"commanded_steering_norm"`

	ActiveManeuverType *string  `json:"active_maneuver_type"`
	ManeuverSteering   *float64 `json:"maneuver_steering"`
	ManeuverSpeedMPS   *float64 `json:"maneuver_speed_mps"`
	ManeuverFramesLeft *int     `json:"maneuver_frames_left"`
	EscapeCount        *int     `json:"escape_count"`

	ParkingEngaged *bool   `json:"parking_engaged"`
	ParkPhase      *string `json:"park_phase"`

	ActiveSignCount      *int     `json:"active_sign_count"`
	SignDeformMagnitudeM *float64 `json:"sign_deform_magnitude_m"`

	DirectionGateVerdict           *string  `json:"direction_gate_verdict"`
	DirectionLeftRangeM            *float64 `json:"direction_left_range_m"`
	DirectionRightRangeM           *float64 `json:"direction_right_range_m"`
	DirectionVotesClockwise        *int     `json:"direction_votes_clockwise"`
	DirectionVotesCounterclockwise *int     `json:"direction_votes_counterclockwise"`
	CorridorWidthBeliefM           *float64 `json:"corridor_width_belief_m"`
}

// stringerPtr renders an optional enum as an optional wire string. Returns
// nil for a nil input so an unset field marshals as null rather than as the
// zero enum's name, which would read as a real observation.
func stringerPtr[T interface{ String() string }](v *T) *string {
	if v == nil {
		return nil
	}
	s := (*v).String()
	return &s
}

// MarshalWireJSON renders the snapshot as the /nav_debug payload, the same
// document CoreNavigator publishes and every diag_bag_*.py script parses.
//
// This is what lets the existing Python bag-analysis suite run on a
// simulated run: the scripts key off the field names below, so a sim bag
// that carries them is indistinguishable to them from a track recording.
func (d DebugSnapshot) MarshalWireJSON() ([]byte, error) {
	return json.Marshal(wireSnapshot{
		Phase: d.Phase.String(),

		PoseX:           d.PoseX,
		PoseY:           d.PoseY,
		PoseYaw:         d.PoseYaw,
		Direction:       stringerPtr(d.Direction),
		CurrentCorridor: stringerPtr(d.CurrentCorridor),
		WaypointIndex:   d.WaypointIndex,
		LapsCompleted:   d.LapsCompleted,
		NumLaps:         d.NumLaps,

		IsStuck:         d.IsStuck,
		StuckCount:      d.StuckCount,
		RecentMovementM: d.RecentMovementM,

		ForwardClearanceM: d.ForwardClearanceM,
		MinLidarRangeM:    d.MinLidarRangeM,
		Risk:              stringerPtr(d.Risk),
		EscapeRisk:        stringerPtr(d.EscapeRisk),
		RearClearanceM:    d.RearClearanceM,

		CrosstrackErrorM:   d.CrosstrackErrorM,
		LookaheadDistanceM: d.LookaheadDistance,
		PathTurnAheadRad:   d.PathTurnAheadRad,
		SteerTargetX:       d.SteerTargetX,
		SteerTargetY:       d.SteerTargetY,
		AngleErrorRad:      d.AngleErrorRad,

		ClearanceSpeedMPS: d.ClearanceSpeedMPS,
		HeadingSpeedMPS:   d.HeadingSpeedMPS,

		CommandedSpeedMPS:     d.CommandedSpeedMPS,
		CommandedSteeringNorm: d.CommandedSteerNorm,

		ActiveManeuverType: stringerPtr(d.ActiveManeuverType),
		ManeuverSteering:   d.ManeuverSteering,
		ManeuverSpeedMPS:   d.ManeuverSpeedMPS,
		ManeuverFramesLeft: d.ManeuverFramesLeft,
		EscapeCount:        d.EscapeCount,

		ParkPhase: stringerPtr(d.ParkPhase),

		ActiveSignCount:      d.ActiveSignCount,
		SignDeformMagnitudeM: d.SignDeformMagnitudeM,
	})
}
