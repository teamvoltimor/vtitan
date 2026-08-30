package navigator

import (
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	navv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/nav/v1"
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
	}
	if d.WaypointIndex != nil {
		wi := int32(*d.WaypointIndex)
		out.Race.WaypointIndex = &wi
	}
	if d.IsStuck != nil || d.StuckCount != nil || d.RecentMovementM != nil {
		out.Stuck = &navv1.StuckDetection{}
		if d.IsStuck != nil {
			v := *d.IsStuck
			out.Stuck.IsStuck = &v
		}
		if d.StuckCount != nil {
			v := int32(*d.StuckCount)
			out.Stuck.StuckCount = &v
		}
		if d.RecentMovementM != nil {
			v := *d.RecentMovementM
			out.Stuck.RecentMovementM = &v
		}
	}
	out.Perception = &navv1.Perception{
		Risk:       navv1.RiskLevel(deoptRisk(d.Risk)) + 1,
		EscapeRisk: navv1.RiskLevel(deoptRisk(d.EscapeRisk)) + 1,
	}
	if d.ForwardClearanceM != nil {
		v := *d.ForwardClearanceM
		out.Perception.ForwardClearanceM = &v
	}
	if d.MinLidarRangeM != nil {
		v := *d.MinLidarRangeM
		out.Perception.MinLidarRangeM = &v
	}
	if d.RearClearanceM != nil {
		v := *d.RearClearanceM
		out.Perception.RearClearanceM = &v
	}
	out.PathTracking = &navv1.PathTracking{}
	if d.CrosstrackErrorM != nil {
		v := *d.CrosstrackErrorM
		out.PathTracking.CrosstrackErrorM = &v
	}
	if d.LookaheadDistance != nil {
		v := *d.LookaheadDistance
		out.PathTracking.LookaheadDistanceM = &v
	}
	if d.PathTurnAheadRad != nil {
		v := *d.PathTurnAheadRad
		out.PathTracking.PathTurnAheadRad = &v
	}
	if d.SteerTargetX != nil {
		v := *d.SteerTargetX
		out.PathTracking.SteerTargetX = &v
	}
	if d.SteerTargetY != nil {
		v := *d.SteerTargetY
		out.PathTracking.SteerTargetY = &v
	}
	if d.AngleErrorRad != nil {
		v := *d.AngleErrorRad
		out.PathTracking.AngleErrorRad = &v
	}
	out.Speed = &navv1.SpeedSelection{}
	if d.ClearanceSpeedMPS != nil {
		v := *d.ClearanceSpeedMPS
		out.Speed.ClearanceSpeedMps = &v
	}
	if d.HeadingSpeedMPS != nil {
		v := *d.HeadingSpeedMPS
		out.Speed.HeadingSpeedMps = &v
	}
	out.Command = &navv1.DriveCommand{}
	if d.CommandedSpeedMPS != nil {
		v := *d.CommandedSpeedMPS
		out.Command.SpeedMps = &v
	}
	if d.CommandedSteerNorm != nil {
		v := *d.CommandedSteerNorm
		out.Command.SteeringNorm = &v
	}
	out.Maneuver = &navv1.Maneuver{
		ActiveType: navv1.ManeuverType(deoptManeuver(d.ActiveManeuverType)) + 1,
	}
	if d.ManeuverSteering != nil {
		v := *d.ManeuverSteering
		out.Maneuver.Steering = &v
	}
	if d.ManeuverSpeedMPS != nil {
		v := *d.ManeuverSpeedMPS
		out.Maneuver.SpeedMps = &v
	}
	if d.ManeuverFramesLeft != nil {
		v := int32(*d.ManeuverFramesLeft)
		out.Maneuver.FramesLeft = &v
	}
	if d.EscapeCount != nil {
		v := int32(*d.EscapeCount)
		out.Maneuver.EscapeCount = &v
	}
	out.SignRouting = &navv1.SignRouting{}
	if d.ActiveSignCount != nil {
		v := int32(*d.ActiveSignCount)
		out.SignRouting.ActiveSignCount = &v
	}
	if d.SignDeformMagnitudeM != nil {
		v := *d.SignDeformMagnitudeM
		out.SignRouting.DeformMagnitudeM = &v
	}
	return out
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
