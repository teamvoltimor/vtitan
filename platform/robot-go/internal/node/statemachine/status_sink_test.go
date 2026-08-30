package statemachine_test

import (
	"testing"

	"buf.build/go/protovalidate"

	nodestatemachine "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/statemachine"
	statev1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/state/v1"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	smcore "github.com/teamvoltimor/vtitan/platform/robot-go/internal/statemachine/core"
)

// TestRaceMetricsMessageFor_CarriesDegreesUnchanged pins the unit decision:
// RaceStatus reports steering and yaw in DEGREES, and this conversion must
// pass them through rather than "helpfully" normalizing or converting to
// radians -- the schema field names now assert degrees.
func TestRaceMetricsMessageFor_CarriesDegreesUnchanged(t *testing.T) {
	t.Parallel()

	corridor := "north"
	targetLaps := 3

	got := nodestatemachine.RaceMetricsMessageFor(smcore.RaceStatus{
		CurrentCorridor:    &corridor,
		TotalRaceTimeSec:   42.5,
		CurrentVelocityMPS: -0.25,
		CurrentSteeringDeg: 37.0,
		GyroYawDeg:         -190.0,
		LapsCompleted:      2,
	}, &targetLaps)

	if got.GetCurrentSteeringDeg() != 37.0 {
		t.Fatalf("CurrentSteeringDeg = %v, want 37 passed through unchanged", got.GetCurrentSteeringDeg())
	}
	// Unwrapped yaw must survive: a producer reporting one should surface it.
	if got.GetGyroYawDeg() != -190.0 {
		t.Fatalf("GyroYawDeg = %v, want -190 kept unwrapped", got.GetGyroYawDeg())
	}
	if got.GetCurrentVelocityMps() != -0.25 {
		t.Fatalf("CurrentVelocityMps = %v, want -0.25 (reverse is normal)", got.GetCurrentVelocityMps())
	}
	if got.GetTargetLaps() != 3 || got.GetLapsCompleted() != 2 {
		t.Fatalf("laps = %d/%d, want 2/3", got.GetLapsCompleted(), got.GetTargetLaps())
	}
	if got.GetCurrentCorridor() != "north" {
		t.Fatalf("CurrentCorridor = %q, want %q", got.GetCurrentCorridor(), "north")
	}
}

// TestRaceMetricsMessageFor_AbsentOptionals covers the distinction the wire
// model exists to preserve: no lap target yet is NOT a target of zero, and
// an unknown corridor is not a corridor named "".
func TestRaceMetricsMessageFor_AbsentOptionals(t *testing.T) {
	t.Parallel()

	got := nodestatemachine.RaceMetricsMessageFor(smcore.RaceStatus{LapsCompleted: 1}, nil)

	if got.TargetLaps != nil {
		t.Fatalf("TargetLaps = %v, want absent so the OLED falls back to its own target", *got.TargetLaps)
	}
	if got.GetCurrentCorridor() != "" {
		t.Fatalf("CurrentCorridor = %q, want empty when unclassified", got.GetCurrentCorridor())
	}
}

// TestSystemStatusMessageFor covers the DiagnosticArray shaping: one entry
// per sensor plus a rollup, with not-ready reported as blocking.
func TestSystemStatusMessageFor(t *testing.T) {
	t.Parallel()

	scenario := smcore.ScenarioObstacles
	got := nodestatemachine.SystemStatusMessageFor(smcore.SystemStatus{
		NetworkStatus: "up",
		ChallengeMode: &scenario,
		IMUStatus:     smcore.SensorStatus{Name: "bno085", IsReady: true},
		LidarStatus:   smcore.SensorStatus{IsReady: false, ErrorMessage: "no scan"},
		AllReady:      false,
	})

	if len(got.GetStatus()) != 6 {
		t.Fatalf("got %d status entries, want 5 sensors + 1 rollup", len(got.GetStatus()))
	}

	var lidar, rollup *statev1.SystemStatus_Status
	for _, entry := range got.GetStatus() {
		switch entry.GetHardwareId() {
		case "lidar":
			lidar = entry
		case "":
			rollup = entry
		}
	}

	if lidar == nil {
		t.Fatal("no lidar entry")
	}
	// Boot check gates the race starting at all, so not-ready is blocking.
	if lidar.GetLevel() != statev1.SystemStatus_LEVEL_ERROR {
		t.Fatalf("lidar level = %v, want LEVEL_ERROR", lidar.GetLevel())
	}
	// SensorStatus.Name was empty, so the fixed key must fill in -- the
	// schema requires a non-empty name.
	if lidar.GetName() != "lidar" {
		t.Fatalf("lidar name = %q, want the fallback key", lidar.GetName())
	}
	if lidar.GetMessage() != "no scan" {
		t.Fatalf("lidar message = %q, want the error text", lidar.GetMessage())
	}

	if rollup == nil {
		t.Fatal("no rollup entry")
	}
	if len(rollup.GetValues()) != 1 || rollup.GetValues()[0].GetValue() != "obstacles" {
		t.Fatalf("rollup values = %v, want the resolved challenge mode", rollup.GetValues())
	}
}

// TestSystemStatusMessageFor_PassesValidation guards the name fallback: the
// schema requires every entry to be named, so an unnamed SensorStatus must
// not be able to produce a message that fails at the publisher.
func TestSystemStatusMessageFor_PassesValidation(t *testing.T) {
	t.Parallel()

	validator, err := protovalidate.New()
	if err != nil {
		t.Fatalf("protovalidate.New: %v", err)
	}

	got := nodestatemachine.SystemStatusMessageFor(smcore.SystemStatus{})
	if validateErr := validator.Validate(got); validateErr != nil {
		t.Fatalf("Validate() = %v, want nil even with every SensorStatus unnamed", validateErr)
	}
}

func TestChallengeModeMessageFor(t *testing.T) {
	t.Parallel()

	tests := map[string]struct {
		scenario smcore.ScenarioType
		want     uiv1.Challenge
	}{
		"open":      {smcore.ScenarioOpen, uiv1.Challenge_CHALLENGE_OPEN},
		"obstacles": {smcore.ScenarioObstacles, uiv1.Challenge_CHALLENGE_OBSTACLES},
	}

	for name, tt := range tests {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			if got := nodestatemachine.ChallengeModeMessageFor(tt.scenario).GetChallenge(); got != tt.want {
				t.Fatalf("Challenge = %v, want %v", got, tt.want)
			}
		})
	}
}

// TestJumperInsertedMessageFor covers the raw reading staying separate from
// the resolved mode -- "no jumper, defaulting to Open" must remain
// distinguishable from "jumper fitted, selecting Open".
func TestJumperInsertedMessageFor(t *testing.T) {
	t.Parallel()

	if got := nodestatemachine.JumperInsertedMessageFor(false); got.GetInserted() {
		t.Fatal("Inserted = true, want false")
	}
	if got := nodestatemachine.JumperInsertedMessageFor(true); !got.GetInserted() {
		t.Fatal("Inserted = false, want true")
	}
}
