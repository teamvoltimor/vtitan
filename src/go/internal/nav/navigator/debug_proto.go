package navigator

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
)

// ToProto maps the per-tick DebugSnapshot to the wire navigator_debug proto, so
// a Go run recorded with --record can be replayed by the same bagreplay parity
// harness that reads Python-baseline bags (test/bagreplay). Pointer fields stay
// nil when the snapshot's branch didn't compute them, preserving the
// "nil means not-on-this-phase" distinction the Python side encodes as
// float | None.
func (d DebugSnapshot) ToProto() *navv1.NavigatorDebug {
	out := &navv1.NavigatorDebug{
		Phase: navv1.Phase(d.Phase) + 1, // PhaseNotYetStepped(0) -> PHASE_NOT_YET_STEPPED(1)
	}
	if d.PoseX != nil && d.PoseY != nil && d.PoseYaw != nil {
		out.Pose = &navv1.NavigatorPose{
			X:   *d.PoseX,
			Y:   *d.PoseY,
			Yaw: *d.PoseYaw,
		}
	}
	out.Race = &navv1.RaceState{
		Direction:       navv1.Direction(deoptDir(d.Direction)) + 1,
		CurrentCorridor: navv1.Section(deoptSection(d.CurrentCorridor)) + 1,
		LapsCompleted:   int32(d.LapsCompleted),
		NumLaps:         int32(d.NumLaps),
		WaypointIndex:   protoInt32(d.WaypointIndex),
	}
	if d.IsStuck != nil || d.StuckCount != nil || d.RecentMovementM != nil {
		out.Stuck = &navv1.StuckDetection{
			IsStuck:         d.IsStuck,
			StuckCount:      protoInt32(d.StuckCount),
			RecentMovementM: protoFloat64(d.RecentMovementM),
		}
	}
	out.Perception = &navv1.Perception{
		Risk:              navv1.RiskLevel(deoptRisk(d.Risk)) + 1,
		EscapeRisk:        navv1.RiskLevel(deoptRisk(d.EscapeRisk)) + 1,
		ForwardClearanceM: protoFloat64(d.ForwardClearanceM),
		MinLidarRangeM:    protoFloat64(d.MinLidarRangeM),
		RearClearanceM:    protoFloat64(d.RearClearanceM),
	}
	out.PathTracking = &navv1.PathTracking{
		CrosstrackErrorM:   protoFloat64(d.CrosstrackErrorM),
		LookaheadDistanceM: protoFloat64(d.LookaheadDistance),
		PathTurnAheadRad:   protoFloat64(d.PathTurnAheadRad),
		SteerTargetX:       protoFloat64(d.SteerTargetX),
		SteerTargetY:       protoFloat64(d.SteerTargetY),
		AngleErrorRad:      protoFloat64(d.AngleErrorRad),
	}
	out.Speed = &navv1.SpeedSelection{
		ClearanceSpeedMps: protoFloat64(d.ClearanceSpeedMPS),
		HeadingSpeedMps:   protoFloat64(d.HeadingSpeedMPS),
	}
	out.Command = &navv1.DriveCommand{
		SpeedMps:     protoFloat64(d.CommandedSpeedMPS),
		SteeringNorm: protoFloat64(d.CommandedSteerNorm),
	}
	out.Maneuver = &navv1.Maneuver{
		ActiveType:  navv1.ManeuverType(deoptManeuver(d.ActiveManeuverType)) + 1,
		Steering:    protoFloat64(d.ManeuverSteering),
		SpeedMps:    protoFloat64(d.ManeuverSpeedMPS),
		FramesLeft:  protoInt32(d.ManeuverFramesLeft),
		EscapeCount: protoInt32(d.EscapeCount),
	}
	out.SignRouting = &navv1.SignRouting{
		ActiveSignCount:  protoInt32(d.ActiveSignCount),
		DeformMagnitudeM: protoFloat64(d.SignDeformMagnitudeM),
	}
	return out
}

// protoFloat64 and protoInt32 copy an optional snapshot field into its wire
// representation, returning nil when unset so the proto keeps the
// not-on-this-phase distinction.
func protoFloat64(v *float64) *float64 {
	if v == nil {
		return nil
	}
	x := *v
	return &x
}

func protoInt32(v *int) *int32 {
	if v == nil {
		return nil
	}
	x := int32(*v)
	return &x
}

// deopt* return the zero value of an optional enum field when the pointer is nil,
// so the +1 proto offset lands on the UNSPECIFIED value instead of an out-of-range
// constant.
func deoptDir(d *trackmodel.Direction) int {
	if d == nil {
		return 0
	}
	return int(*d)
}

func deoptSection(s *trackmodel.Section) int {
	if s == nil {
		return 0
	}
	return int(*s)
}

func deoptManeuver(m *controllers.ManeuverType) int {
	if m == nil {
		return 0
	}
	return int(*m)
}

func deoptRisk(r *controllers.RiskLevel) int {
	if r == nil {
		return 0
	}
	return int(*r)
}
