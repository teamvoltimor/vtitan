package bagreplay_test

import (
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/recording"
	"github.com/teamvoltimor/vtitan/src/go/test/bagreplay"
)

// TestNavDebugWire_RoundTripsThroughTheBagReader is the contract that makes
// a simulated run readable by the PYTHON bag-analysis suite.
//
// The encoder (navigator.DebugSnapshot.MarshalWireJSON) and the decoder
// (bagreplay.NavDebugSnapshot) are two independent statements of the same
// wire format -- one written from the producer's side, one reverse-engineered
// from real recordings off the robot. Round-tripping proves they agree,
// which is the only reason a diag_bag_*.py script can be pointed at a sim
// bag and be trusted. A field renamed on one side alone fails here.
func TestNavDebugWire_RoundTripsThroughTheBagReader(t *testing.T) {
	t.Parallel()

	snap := navigator.DebugSnapshot{
		Phase:              navigator.PhaseNormalDrive,
		PoseX:              new(1.25),
		PoseY:              new(0.303),
		PoseYaw:            new(3.14),
		Direction:          new(trackmodel.Clockwise),
		CurrentCorridor:    new(trackmodel.South),
		WaypointIndex:      new(7),
		LapsCompleted:      2,
		NumLaps:            3,
		IsStuck:            new(false),
		StuckCount:         new(1),
		RecentMovementM:    new(0.42),
		ForwardClearanceM:  new(1.1),
		MinLidarRangeM:     new(0.15),
		Risk:               new(controllers.RiskObstacle),
		CrosstrackErrorM:   new(-0.06),
		LookaheadDistance:  new(0.32),
		AngleErrorRad:      new(0.21),
		CommandedSpeedMPS:  new(0.38),
		CommandedSteerNorm: new(-0.5),
		EscapeCount:        new(4),
	}

	raw, err := snap.MarshalWireJSON()
	if err != nil {
		t.Fatalf("MarshalWireJSON: %v", err)
	}
	// Through the std_msgs/String envelope the real topic carries, so the
	// test exercises the same path a recorded message takes.
	got, err := bagreplay.DecodeNavDebug(recording.EncodeString(string(raw)))
	if err != nil {
		t.Fatalf("DecodeNavDebug on our own encoding: %v", err)
	}

	if got.Phase != "normal_drive" {
		t.Errorf("phase = %q, want %q", got.Phase, "normal_drive")
	}
	if got.PoseX == nil || *got.PoseX != 1.25 {
		t.Errorf("pose_x = %v, want 1.25", got.PoseX)
	}
	if got.Direction == nil || *got.Direction != "clockwise" {
		t.Errorf("direction = %v, want clockwise", got.Direction)
	}
	if got.CurrentCorridor == nil || *got.CurrentCorridor != "south" {
		t.Errorf("current_corridor = %v, want south", got.CurrentCorridor)
	}
	if got.Risk == nil || *got.Risk != "obstacle" {
		t.Errorf("risk = %v, want obstacle", got.Risk)
	}
	if got.LapsCompleted != 2 || got.NumLaps != 3 {
		t.Errorf("laps = %d/%d, want 2/3", got.LapsCompleted, got.NumLaps)
	}
	if got.CommandedSteeringNorm == nil || *got.CommandedSteeringNorm != -0.5 {
		t.Errorf("commanded_steering_norm = %v, want -0.5", got.CommandedSteeringNorm)
	}
	if got.LookaheadDistanceM == nil || *got.LookaheadDistanceM != 0.32 {
		t.Errorf("lookahead_distance_m = %v, want 0.32", got.LookaheadDistanceM)
	}
	// Fields the Go navigator does not produce must arrive as ABSENT, not as
	// a zero that reads like a real measurement.
	if got.CorridorWidthBeliefM != nil {
		t.Errorf("corridor_width_belief_m = %v, want nil (Go does not report it)", got.CorridorWidthBeliefM)
	}
}
