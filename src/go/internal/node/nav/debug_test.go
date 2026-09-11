package nav_test

import (
	"testing"

	"buf.build/go/protovalidate"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/nav"
	navv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/nav/v1"
)

// TestDebugFor_NoPose is the branch that matters most for presence: the
// no-pose tick publishes a real zero command and nothing else. The zeros
// must survive as a SET command group while pose and race stay absent --
// collapsing either direction would misreport a deliberate stop.
func TestDebugFor_NoPose(t *testing.T) {
	t.Parallel()

	got := nav.DebugFor(navigator.DebugSnapshot{
		Phase:              navigator.PhaseNoPose,
		CommandedSpeedMPS:  new(0.0),
		CommandedSteerNorm: new(0.0),
	})

	if got.GetPhase() != navv1.Phase_PHASE_NO_POSE {
		t.Fatalf("Phase = %v, want PHASE_NO_POSE", got.GetPhase())
	}
	if got.GetPose() != nil {
		t.Fatalf("Pose = %v, want nil on the no-pose branch", got.GetPose())
	}
	if got.GetRace() != nil {
		t.Fatalf("Race = %v, want nil when no pose was available", got.GetRace())
	}
	if got.GetCommand() == nil {
		t.Fatal("Command = nil, want a set group carrying the deliberate stop")
	}
	if got.GetCommand().GetSpeedMps() != 0 || got.GetCommand().GetSteeringNorm() != 0 {
		t.Fatalf("Command = %v, want zero speed and steering", got.GetCommand())
	}
	// The stop is a real command, so the fields must be PRESENT zeros rather
	// than absent ones.
	if got.GetCommand().SpeedMps == nil || got.GetCommand().SteeringNorm == nil {
		t.Fatal("command fields are absent, want present zeros")
	}
}

// TestDebugFor_AbsentGroupsStayAbsent covers the whole point of the nested
// schema: a group that did not run must not appear as an empty message
// asserting "ran, found nothing".
func TestDebugFor_AbsentGroupsStayAbsent(t *testing.T) {
	t.Parallel()

	got := nav.DebugFor(navigator.DebugSnapshot{Phase: navigator.PhaseNotYetStepped})

	if got.GetPose() != nil || got.GetRace() != nil || got.GetStuck() != nil ||
		got.GetPerception() != nil || got.GetPathTracking() != nil ||
		got.GetSpeed() != nil || got.GetCommand() != nil ||
		got.GetManeuver() != nil || got.GetSignRouting() != nil {
		t.Fatalf("empty snapshot produced populated groups: %v", got)
	}
	// Never populated: no ParkController, no blind-mode bootstrap.
	if got.GetParking() != nil || got.GetBlindCreep() != nil {
		t.Fatal("parking/blind-creep groups set, but neither has a Go counterpart")
	}
}

// TestDebugFor_NormalDrive checks the fully-populated path, including that
// signed values keep their sign and that int widths survive the conversion.
func TestDebugFor_NormalDrive(t *testing.T) {
	t.Parallel()

	direction := trackmodel.Counterclockwise
	corridor := trackmodel.West
	risk := controllers.RiskObstacle
	escapeRisk := controllers.RiskCritical

	got := nav.DebugFor(navigator.DebugSnapshot{
		Phase:              navigator.PhaseNormalDrive,
		PoseX:              new(1.25),
		PoseY:              new(2.5),
		PoseYaw:            new(-0.75),
		Direction:          &direction,
		CurrentCorridor:    &corridor,
		WaypointIndex:      new(7),
		LapsCompleted:      2,
		NumLaps:            3,
		ForwardClearanceM:  new(0.42),
		Risk:               &risk,
		EscapeRisk:         &escapeRisk,
		CrosstrackErrorM:   new(-0.08),
		ClearanceSpeedMPS:  new(0.31),
		CommandedSpeedMPS:  new(0.29),
		CommandedSteerNorm: new(-0.4),
	})

	if got.GetPose().GetYaw() != -0.75 {
		t.Fatalf("Yaw = %v, want -0.75", got.GetPose().GetYaw())
	}
	if got.GetRace().GetDirection() != navv1.Direction_DIRECTION_COUNTERCLOCKWISE {
		t.Fatalf("Direction = %v, want COUNTERCLOCKWISE", got.GetRace().GetDirection())
	}
	if got.GetRace().GetCurrentCorridor() != navv1.Section_SECTION_WEST {
		t.Fatalf("CurrentCorridor = %v, want WEST", got.GetRace().GetCurrentCorridor())
	}
	if got.GetRace().GetWaypointIndex() != 7 || got.GetRace().GetLapsCompleted() != 2 {
		t.Fatalf("race = %v, want waypoint 7 and 2 laps", got.GetRace())
	}
	if got.GetPerception().GetRisk() != navv1.RiskLevel_RISK_LEVEL_OBSTACLE ||
		got.GetPerception().GetEscapeRisk() != navv1.RiskLevel_RISK_LEVEL_CRITICAL {
		t.Fatalf("risk = %v, want the two readings kept distinct", got.GetPerception())
	}
	// Signed: which side of the path the robot is on is the point.
	if got.GetPathTracking().GetCrosstrackErrorM() != -0.08 {
		t.Fatalf("CrosstrackErrorM = %v, want -0.08", got.GetPathTracking().GetCrosstrackErrorM())
	}
	// Perception ran but measured no min range; that field stays absent even
	// though its group is present.
	if got.GetPerception().MinLidarRangeM != nil {
		t.Fatal("MinLidarRangeM is set, want absent within a present group")
	}
	if got.GetManeuver() != nil {
		t.Fatal("Maneuver set on a normal-drive tick")
	}
}

// TestPhaseMapping_IsExhaustive guards the failure this conversion is most
// likely to hide: a phase added to the domain enum and never mapped, which
// would silently publish PHASE_UNSPECIFIED forever.
func TestPhaseMapping_IsExhaustive(t *testing.T) {
	t.Parallel()

	seen := make([]bool, len(navv1.Phase_name))
	for phase := navigator.PhaseNotYetStepped; phase <= navigator.PhaseEscapeTriggered; phase++ {
		got := nav.DebugFor(navigator.DebugSnapshot{Phase: phase}).GetPhase()
		if got == navv1.Phase_PHASE_UNSPECIFIED {
			t.Fatalf("phase %v (%d) maps to PHASE_UNSPECIFIED", phase, phase)
		}
		if seen[int(got)] {
			t.Fatalf("phase %v maps to %v, already used by an earlier phase", phase, got)
		}
		seen[int(got)] = true
	}
}

// TestDebugFor_DoesNotAliasSnapshot matters because the navigator reuses its
// snapshot between ticks: a wire message pointing into it would mutate after
// being handed to the publisher.
func TestDebugFor_DoesNotAliasSnapshot(t *testing.T) {
	t.Parallel()

	speed := 0.5
	snapshot := navigator.DebugSnapshot{Phase: navigator.PhaseNormalDrive, CommandedSpeedMPS: &speed}

	got := nav.DebugFor(snapshot)
	speed = 99.0

	if got.GetCommand().GetSpeedMps() != 0.5 {
		t.Fatalf("SpeedMps = %v, want 0.5 -- the message aliased the snapshot",
			got.GetCommand().GetSpeedMps())
	}
}

// TestDebugFor_PassesValidation ties the conversion to the schema's own
// constraints: a converter that produced an out-of-range steering value or
// dropped the stamp would be rejected on the wire, not here.
func TestDebugFor_PassesValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	got := nav.DebugFor(navigator.DebugSnapshot{
		Phase:              navigator.PhaseNormalDrive,
		PoseX:              new(1.0),
		PoseY:              new(1.0),
		PoseYaw:            new(0.0),
		CommandedSpeedMPS:  new(-0.25),
		CommandedSteerNorm: new(-1.0),
	})

	if validateErr := validator.Validate(got); validateErr != nil {
		t.Fatalf("Validate() = %v, want nil", validateErr)
	}
}

func TestLapsCompletedAndCorridor(t *testing.T) {
	t.Parallel()

	if got := nav.LapsCompletedFor(3).GetLapsCompleted(); got != 3 {
		t.Fatalf("LapsCompleted = %d, want 3", got)
	}
	if got := nav.CurrentCorridorFor(trackmodel.East).GetSection(); got != navv1.Section_SECTION_EAST {
		t.Fatalf("Section = %v, want EAST", got)
	}
}
