package nav

import (
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	navv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/nav/v1"
)

// FrameID is the frame every nav message is stamped with, matching
// internal/node/motor's FrameID.
const FrameID = "base_link"

// DebugFor converts a per-tick navigator snapshot into its wire form.
//
// Each sub-message is built only when the snapshot actually carries that
// group, so an absent group means "that branch of Step did not run this
// tick" -- see this package's doc.go on why that matters.
func DebugFor(snapshot navigator.DebugSnapshot) *navv1.NavigatorDebug {
	return &navv1.NavigatorDebug{
		Stamp:        timestamppb.Now(),
		FrameId:      FrameID,
		Phase:        phaseFor(snapshot.Phase),
		Pose:         poseFor(snapshot),
		Race:         raceFor(snapshot),
		Stuck:        stuckFor(snapshot),
		Perception:   perceptionFor(snapshot),
		PathTracking: pathTrackingFor(snapshot),
		Speed:        speedFor(snapshot),
		Command:      commandFor(snapshot),
		Maneuver:     maneuverFor(snapshot),
		SignRouting:  signRoutingFor(snapshot),
		// Parking and BlindCreep are deliberately never set -- see doc.go.
	}
}

// LapsCompletedFor converts a confirmed lap count into its wire form.
func LapsCompletedFor(lapsCompleted int) *navv1.LapsCompleted {
	return &navv1.LapsCompleted{
		Stamp:         timestamppb.Now(),
		LapsCompleted: int32(lapsCompleted),
	}
}

// CurrentCorridorFor converts the navigator's corridor classification into
// its wire form.
func CurrentCorridorFor(section trackmodel.Section) *navv1.CurrentCorridor {
	return &navv1.CurrentCorridor{
		Stamp:   timestamppb.Now(),
		Section: sectionFor(&section),
	}
}

// poseFor returns the pose group, or nil on the no-pose branch.
//
// Requires all three components rather than any one: the navigator's
// _base_debug sets them together or not at all, so a partial pose would mean
// the producer changed shape, not that a component was unmeasurable.
func poseFor(snapshot navigator.DebugSnapshot) *navv1.NavigatorPose {
	if snapshot.PoseX == nil || snapshot.PoseY == nil || snapshot.PoseYaw == nil {
		return nil
	}
	return &navv1.NavigatorPose{
		X:   *snapshot.PoseX,
		Y:   *snapshot.PoseY,
		Yaw: *snapshot.PoseYaw,
	}
}

// raceFor returns the race-state group, or nil where no pose was available.
//
// LapsCompleted and NumLaps are plain ints with no absent form, so presence
// is decided by the pointer fields _base_debug sets alongside them; without
// that test a no-pose tick would emit a group claiming zero laps of zero.
func raceFor(snapshot navigator.DebugSnapshot) *navv1.RaceState {
	if snapshot.Direction == nil && snapshot.WaypointIndex == nil && snapshot.CurrentCorridor == nil {
		return nil
	}
	return &navv1.RaceState{
		Direction:       directionFor(snapshot.Direction),
		CurrentCorridor: sectionFor(snapshot.CurrentCorridor),
		WaypointIndex:   optInt32(snapshot.WaypointIndex),
		LapsCompleted:   int32(snapshot.LapsCompleted),
		NumLaps:         int32(snapshot.NumLaps),
	}
}

func stuckFor(snapshot navigator.DebugSnapshot) *navv1.StuckDetection {
	if snapshot.IsStuck == nil && snapshot.StuckCount == nil && snapshot.RecentMovementM == nil {
		return nil
	}
	return &navv1.StuckDetection{
		IsStuck:         opt(snapshot.IsStuck),
		StuckCount:      optInt32(snapshot.StuckCount),
		RecentMovementM: opt(snapshot.RecentMovementM),
	}
}

func perceptionFor(snapshot navigator.DebugSnapshot) *navv1.Perception {
	if snapshot.ForwardClearanceM == nil && snapshot.MinLidarRangeM == nil &&
		snapshot.Risk == nil && snapshot.EscapeRisk == nil && snapshot.RearClearanceM == nil {
		return nil
	}
	return &navv1.Perception{
		ForwardClearanceM: opt(snapshot.ForwardClearanceM),
		MinLidarRangeM:    opt(snapshot.MinLidarRangeM),
		Risk:              riskFor(snapshot.Risk),
		EscapeRisk:        riskFor(snapshot.EscapeRisk),
		RearClearanceM:    opt(snapshot.RearClearanceM),
	}
}

func pathTrackingFor(snapshot navigator.DebugSnapshot) *navv1.PathTracking {
	if snapshot.CrosstrackErrorM == nil && snapshot.LookaheadDistance == nil &&
		snapshot.PathTurnAheadRad == nil && snapshot.SteerTargetX == nil &&
		snapshot.SteerTargetY == nil && snapshot.AngleErrorRad == nil {
		return nil
	}
	return &navv1.PathTracking{
		CrosstrackErrorM:   opt(snapshot.CrosstrackErrorM),
		LookaheadDistanceM: opt(snapshot.LookaheadDistance),
		PathTurnAheadRad:   opt(snapshot.PathTurnAheadRad),
		SteerTargetX:       opt(snapshot.SteerTargetX),
		SteerTargetY:       opt(snapshot.SteerTargetY),
		AngleErrorRad:      opt(snapshot.AngleErrorRad),
	}
}

func speedFor(snapshot navigator.DebugSnapshot) *navv1.SpeedSelection {
	if snapshot.ClearanceSpeedMPS == nil && snapshot.HeadingSpeedMPS == nil {
		return nil
	}
	return &navv1.SpeedSelection{
		ClearanceSpeedMps: opt(snapshot.ClearanceSpeedMPS),
		HeadingSpeedMps:   opt(snapshot.HeadingSpeedMPS),
	}
}

func commandFor(snapshot navigator.DebugSnapshot) *navv1.DriveCommand {
	if snapshot.CommandedSpeedMPS == nil && snapshot.CommandedSteerNorm == nil {
		return nil
	}
	return &navv1.DriveCommand{
		SpeedMps:     opt(snapshot.CommandedSpeedMPS),
		SteeringNorm: opt(snapshot.CommandedSteerNorm),
	}
}

func maneuverFor(snapshot navigator.DebugSnapshot) *navv1.Maneuver {
	if snapshot.ActiveManeuverType == nil && snapshot.ManeuverSteering == nil &&
		snapshot.ManeuverSpeedMPS == nil && snapshot.ManeuverFramesLeft == nil &&
		snapshot.EscapeCount == nil {
		return nil
	}
	return &navv1.Maneuver{
		ActiveType:  maneuverTypeFor(snapshot.ActiveManeuverType),
		Steering:    opt(snapshot.ManeuverSteering),
		SpeedMps:    opt(snapshot.ManeuverSpeedMPS),
		FramesLeft:  optInt32(snapshot.ManeuverFramesLeft),
		EscapeCount: optInt32(snapshot.EscapeCount),
	}
}

func signRoutingFor(snapshot navigator.DebugSnapshot) *navv1.SignRouting {
	if snapshot.ActiveSignCount == nil && snapshot.SignDeformMagnitudeM == nil {
		return nil
	}
	return &navv1.SignRouting{
		ActiveSignCount:  optInt32(snapshot.ActiveSignCount),
		DeformMagnitudeM: opt(snapshot.SignDeformMagnitudeM),
	}
}

// phaseFor maps the domain phase onto the wire enum.
//
// Written as an exhaustive switch rather than an array indexed by Phase: the
// two enums are independent numbering spaces that only happen to be declared
// in the same order today, and an index-based mapping would silently shift
// every phase if either gained a value.
func phaseFor(phase navigator.Phase) navv1.Phase {
	switch phase {
	case navigator.PhaseNotYetStepped:
		return navv1.Phase_PHASE_NOT_YET_STEPPED
	case navigator.PhaseNoPose:
		return navv1.Phase_PHASE_NO_POSE
	case navigator.PhaseBlindCreep:
		return navv1.Phase_PHASE_BLIND_CREEP
	case navigator.PhaseActiveManeuver:
		return navv1.Phase_PHASE_ACTIVE_MANEUVER
	case navigator.PhaseStuckEscapeHolding:
		return navv1.Phase_PHASE_STUCK_ESCAPE_HOLDING
	case navigator.PhaseStuckEscapeManeuver:
		return navv1.Phase_PHASE_STUCK_ESCAPE_MANEUVER
	case navigator.PhaseFinishedHold:
		return navv1.Phase_PHASE_FINISHED_HOLD
	case navigator.PhaseParking:
		return navv1.Phase_PHASE_PARKING
	case navigator.PhaseWaypointWrapFallback:
		return navv1.Phase_PHASE_WAYPOINT_WRAP_FALLBACK
	case navigator.PhaseWaypointReached:
		return navv1.Phase_PHASE_WAYPOINT_REACHED
	case navigator.PhaseNormalDrive:
		return navv1.Phase_PHASE_NORMAL_DRIVE
	case navigator.PhaseEscapeTriggered:
		return navv1.Phase_PHASE_ESCAPE_TRIGGERED
	default:
		return navv1.Phase_PHASE_UNSPECIFIED
	}
}

func directionFor(direction *trackmodel.Direction) navv1.Direction {
	if direction == nil {
		return navv1.Direction_DIRECTION_UNSPECIFIED
	}
	switch *direction {
	case trackmodel.Clockwise:
		return navv1.Direction_DIRECTION_CLOCKWISE
	case trackmodel.Counterclockwise:
		return navv1.Direction_DIRECTION_COUNTERCLOCKWISE
	default:
		return navv1.Direction_DIRECTION_UNSPECIFIED
	}
}

func sectionFor(section *trackmodel.Section) navv1.Section {
	if section == nil {
		return navv1.Section_SECTION_UNSPECIFIED
	}
	switch *section {
	case trackmodel.North:
		return navv1.Section_SECTION_NORTH
	case trackmodel.South:
		return navv1.Section_SECTION_SOUTH
	case trackmodel.East:
		return navv1.Section_SECTION_EAST
	case trackmodel.West:
		return navv1.Section_SECTION_WEST
	default:
		return navv1.Section_SECTION_UNSPECIFIED
	}
}

func riskFor(risk *controllers.RiskLevel) navv1.RiskLevel {
	if risk == nil {
		return navv1.RiskLevel_RISK_LEVEL_UNSPECIFIED
	}
	switch *risk {
	case controllers.RiskSafe:
		return navv1.RiskLevel_RISK_LEVEL_SAFE
	case controllers.RiskObstacle:
		return navv1.RiskLevel_RISK_LEVEL_OBSTACLE
	case controllers.RiskCritical:
		return navv1.RiskLevel_RISK_LEVEL_CRITICAL
	default:
		return navv1.RiskLevel_RISK_LEVEL_UNSPECIFIED
	}
}

func maneuverTypeFor(maneuver *controllers.ManeuverType) navv1.ManeuverType {
	if maneuver == nil {
		return navv1.ManeuverType_MANEUVER_TYPE_UNSPECIFIED
	}
	switch *maneuver {
	case controllers.ManeuverKTurn:
		return navv1.ManeuverType_MANEUVER_TYPE_K_TURN
	case controllers.ManeuverSideCorrection:
		return navv1.ManeuverType_MANEUVER_TYPE_SIDE_CORRECTION
	case controllers.ManeuverStuckReverse:
		return navv1.ManeuverType_MANEUVER_TYPE_STUCK_REVERSE
	case controllers.ManeuverStuckForward:
		return navv1.ManeuverType_MANEUVER_TYPE_STUCK_FORWARD
	default:
		return navv1.ManeuverType_MANEUVER_TYPE_UNSPECIFIED
	}
}

// opt copies a domain pointer into a wire pointer, preserving nil. The copy
// matters: the wire message must not alias the caller's snapshot, which the
// navigator reuses between ticks.
func opt[T any](v *T) *T {
	if v == nil {
		return nil
	}
	return new(*v)
}

// optInt32 is opt for the int-to-int32 width change the wire format needs.
func optInt32(v *int) *int32 {
	if v == nil {
		return nil
	}
	return new(int32(*v))
}
