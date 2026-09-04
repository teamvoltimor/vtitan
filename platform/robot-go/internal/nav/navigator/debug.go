package navigator

import (
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// Phase is the branch of Navigator.Step that produced a snapshot, matching
// shared.domain.enums.NavigatorPhase.
//
// Which branch produced a given DebugSnapshot is what makes a nil field
// mean "not computed on this phase" rather than "unknown" -- see
// DebugSnapshot's own doc comment.
//
// StuckEscapeHolding is mirrored for completeness (it is part of the wire
// format other tooling reads) even though this port never emits it: the
// stuck-escape hold branch it belongs to was replaced by the both-blocked
// pivot. BlindCreep and Parking ARE emitted -- both blind bootstrap and the
// park controller are ported, see doc.go.
type Phase int

// DebugSnapshot is the complete per-tick internal state of Navigator.Step,
// matching shared.domain.models.NavigatorDebugSnapshot.
//
// Diagnosing real-hardware failures meant reconstructing crosstrack error,
// angle error, which lookahead/target were chosen and the risk state by
// hand from raw motor/IMU topics after the fact -- slow, and some internal
// values (crosstrack error, the chosen steer target) cannot be recovered
// from those topics at all. Produced on every control tick regardless of
// which branch Step took, so this is always current rather than
// on-request.
//
// Phase identifies which branch produced the snapshot; a nil pointer field
// means "not computed on this tick's branch", NOT "unknown" or a bug.
// E.g. CrosstrackErrorM is only set on PhaseNormalDrive; it is meaningless
// (and left nil) while an escape maneuver is latched. The pointers are what
// preserve that distinction, which Python gets from `float | None` -- a
// plain 0.0 would read as a real measurement.
//
type DebugSnapshot struct {
	Phase Phase

	// Pose and race state -- available on every phase except PhaseNoPose.
	PoseX           *float64
	PoseY           *float64
	PoseYaw         *float64
	Direction       *trackmodel.Direction
	CurrentCorridor *trackmodel.Section
	WaypointIndex   *int
	LapsCompleted   int
	NumLaps         int

	// Stuck detection -- set whenever the detector runs (see
	// StuckDetector.GetDiagnostics).
	IsStuck         *bool
	StuckCount      *int
	RecentMovementM *float64

	// Perception / risk -- set on PhaseNormalDrive.
	ForwardClearanceM *float64
	MinLidarRangeM    *float64
	Risk              *controllers.RiskLevel
	EscapeRisk        *controllers.RiskLevel
	RearClearanceM    *float64

	// Path tracking -- set on PhaseNormalDrive.
	CrosstrackErrorM  *float64
	LookaheadDistance *float64
	// PathTurnAheadRad is the heading change the planned path makes within
	// the preview distance: ~0 on a straight, rising before a corner, so a
	// bag shows whether the short lookahead armed on entry or a corner late.
	PathTurnAheadRad *float64
	SteerTargetX     *float64
	SteerTargetY     *float64
	AngleErrorRad    *float64

	// Speed selection -- set on PhaseNormalDrive.
	ClearanceSpeedMPS *float64
	HeadingSpeedMPS   *float64

	// Final command -- set on every phase that actually publishes a drive
	// command.
	CommandedSpeedMPS  *float64
	CommandedSteerNorm *float64

	// Escape/stuck maneuver -- set whenever one is latched or begun.
	ActiveManeuverType *controllers.ManeuverType
	ManeuverSteering   *float64
	ManeuverSpeedMPS   *float64
	ManeuverFramesLeft *int
	EscapeCount        *int

	// Obstacles Challenge sign routing -- set on PhaseNormalDrive whenever
	// a SignRouter is attached.
	ActiveSignCount      *int
	SignDeformMagnitudeM *float64

	// ParkPhase is the ParkController phase that produced this tick's
	// command, set on PhaseParking whenever a ParkController is attached
	// (nil for the Open Challenge, which has none).
	ParkPhase *parking.Phase
}

const (
	// PhaseNotYetStepped matches NavigatorPhase.NOT_YET_STEPPED.
	PhaseNotYetStepped Phase = iota
	// PhaseNoPose matches NavigatorPhase.NO_POSE.
	PhaseNoPose
	// PhaseBlindCreep matches NavigatorPhase.BLIND_CREEP.
	PhaseBlindCreep
	// PhaseActiveManeuver matches NavigatorPhase.ACTIVE_MANEUVER.
	PhaseActiveManeuver
	// PhaseStuckEscapeHolding matches NavigatorPhase.STUCK_ESCAPE_HOLDING.
	PhaseStuckEscapeHolding
	// PhaseStuckEscapeManeuver matches NavigatorPhase.STUCK_ESCAPE_MANEUVER.
	PhaseStuckEscapeManeuver
	// PhaseFinishedHold matches NavigatorPhase.FINISHED_HOLD.
	PhaseFinishedHold
	// PhaseParking matches NavigatorPhase.PARKING.
	PhaseParking
	// PhaseWaypointWrapFallback matches NavigatorPhase.WAYPOINT_WRAP_FALLBACK.
	PhaseWaypointWrapFallback
	// PhaseWaypointReached matches NavigatorPhase.WAYPOINT_REACHED.
	PhaseWaypointReached
	// PhaseNormalDrive matches NavigatorPhase.NORMAL_DRIVE.
	PhaseNormalDrive
	// PhaseEscapeTriggered matches NavigatorPhase.ESCAPE_TRIGGERED.
	PhaseEscapeTriggered
)

// phaseNames are the StrEnum values NavigatorPhase carries on the wire, so
// a Go-produced snapshot logs the same token a Python-produced one does.
var phaseNames = [...]string{
	"not_yet_stepped",
	"no_pose",
	"blind_creep",
	"active_maneuver",
	"stuck_escape_holding",
	"stuck_escape_maneuver",
	"finished_hold",
	"parking",
	"waypoint_wrap_fallback",
	"waypoint_reached",
	"normal_drive",
	"escape_triggered",
}

// String returns the wire token for p, matching NavigatorPhase's StrEnum value.
func (p Phase) String() string {
	if p < 0 || int(p) >= len(phaseNames) {
		return "unknown"
	}
	return phaseNames[p]
}
