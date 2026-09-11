package scenario

import (
	"context"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/harness"
)

// inlineOpenMetadata is a minimal valid Open Challenge *_metadata.json for a
// 2 m-wide corridor on every side, robot starting on the SOUTH wall centreline
// travelling counterclockwise. It exercises the native runner end-to-end.
const inlineOpenMetadata = `{
  "challenge_type": "open",
  "scenario_id": 0,
  "num_signs": 0,
  "has_parking_lot": false,
  "corridor_widths": {
    "north": {"type": "wide", "width_mm": 1000},
    "south": {"type": "wide", "width_mm": 1000},
    "east":  {"type": "wide", "width_mm": 1000},
    "west":  {"type": "wide", "width_mm": 1000}
  },
  "starting_conditions": {
    "direction": "counterclockwise",
    "section": "south",
    "position": {"x": 1.5, "y": 0.5},
    "yaw": 1.5707963267948966
  }
}`

// inlineObstaclesMetadata mirrors a real generated Obstacles Challenge
// metadata file (platform/robot/.corpus/obstacles/scenarios/
// scenario_0000_metadata.json): 4 red signs down the south/west corridors,
// a parking lot the native runner now acts on (see NativeRunner's doc
// comment) once the laps finish, robot starting on the north wall
// travelling counterclockwise.
const inlineObstaclesMetadata = `{
  "challenge_type": "obstacles",
  "seed": 2026,
  "parking_lot": {
    "block1_position": {"x": 1.5, "y": 2.9},
    "block2_position": {"x": 1.95, "y": 2.9},
    "block1_yaw": 1.5707963267948966,
    "block2_yaw": 1.5707963267948966,
    "depth": 1.5
  },
  "corridor_widths": {
    "east":  {"type": "fixed", "width_mm": 1000},
    "north": {"type": "fixed", "width_mm": 1000},
    "south": {"type": "fixed", "width_mm": 1000},
    "west":  {"type": "fixed", "width_mm": 1000}
  },
  "sign_positions": [
    {"color": "red", "x": 1,   "y": 0.4},
    {"color": "red", "x": 2,   "y": 0.4},
    {"color": "red", "x": 2.4, "y": 1.5},
    {"color": "red", "x": 0.6, "y": 1}
  ],
  "starting_conditions": {
    "direction": "counterclockwise",
    "section": "North",
    "position": {"x": 1.725, "y": 2.5},
    "yaw": 3.141592653589793
  },
  "scenario_id": 0,
  "num_signs": 4,
  "has_parking_lot": true
}`

func writeTempMetadataString(t *testing.T, raw string) corpus.Scenario {
	t.Helper()
	dir := t.TempDir()
	path := filepath.Join(dir, "scenario_0000_metadata.json")
	if err := os.WriteFile(path, []byte(raw), 0o600); err != nil {
		t.Fatalf("writing temp metadata: %v", err)
	}
	return corpus.Scenario{ID: "scenario_0000", MetadataPath: path}
}

func writeTempMetadata(t *testing.T) corpus.Scenario {
	t.Helper()
	return writeTempMetadataString(t, inlineOpenMetadata)
}

func TestNativeRunnerSmoke(t *testing.T) {
	t.Parallel()
	sc := writeTempMetadata(t)

	cfg := harness.DefaultConfig()
	cfg.LidarNoiseStd = 0.0
	cfg.InvalidRayRate = 0.0
	runner := NewNativeRunner(NativeRunnerConfig{Harness: &cfg, Seed: 1, MaxSteps: 4000})

	res, err := runner.Run(context.Background(), sc)
	if err != nil {
		t.Fatalf("native runner returned error: %v", err)
	}

	if res.Steps == 0 {
		t.Errorf("expected a non-zero Result (Steps > 0), got %+v", res)
	}
	if res.TargetLaps != 3 {
		t.Errorf("expected TargetLaps=3, got %d", res.TargetLaps)
	}
	t.Logf("native smoke: laps=%d/%d collided=%t timed_out=%t stuck=%t steps=%d sim_time=%.1fs",
		res.LapsCompleted, res.TargetLaps, res.Collided, res.TimedOut, res.Stuck, res.Steps, res.SimTimeS)
}

// TestNativeRunnerSmoke_Obstacles exercises the Obstacles Challenge wiring
// end-to-end: signs become LIDAR-visible collision obstacles AND
// SignRouter targets, fed by the visionsim-emulated camera. This is a
// smoke test (it must run without error and produce a populated, internally
// consistent Result), not a Python-parity assertion -- there is no
// Obstacles-corpus parity gate yet, matching NativeRunner's own doc comment
// on what is/isn't validated against the Python oracle so far.
func TestNativeRunnerSmoke_Obstacles(t *testing.T) {
	t.Parallel()
	sc := writeTempMetadataString(t, inlineObstaclesMetadata)

	cfg := harness.DefaultConfig()
	cfg.LidarNoiseStd = 0.0
	cfg.InvalidRayRate = 0.0
	runner := NewNativeRunner(NativeRunnerConfig{Harness: &cfg, Seed: 1, MaxSteps: 4000})

	res, err := runner.Run(context.Background(), sc)
	if err != nil {
		t.Fatalf("native runner returned error: %v", err)
	}

	if res.Steps == 0 {
		t.Fatalf("expected a non-zero Result (Steps > 0), got %+v", res)
	}
	if res.TargetLaps != 3 {
		t.Errorf("expected TargetLaps=3, got %d", res.TargetLaps)
	}
	// PassSideViolationSigns must never name an index outside the 4
	// signs the metadata declares -- a stale/garbage index would mean the
	// router's bookkeeping and the scenario's own sign count have drifted
	// apart.
	for _, idx := range res.PassSideViolationSigns {
		if idx < 0 || idx >= 4 {
			t.Errorf("PassSideViolationSigns contains out-of-range index %d (scenario has 4 signs)", idx)
		}
	}
	if res.PassSideViolation != (len(res.PassSideViolationSigns) > 0) {
		t.Errorf("PassSideViolation=%v inconsistent with PassSideViolationSigns=%v",
			res.PassSideViolation, res.PassSideViolationSigns)
	}
	// This fixture has a parking lot, so ParkController is attached and both
	// fields must be set -- even on a run that collides on lap 0 and never
	// gets near the bay, ScorePark still scores whatever the final pose
	// happens to be (0 points, typically, this far from the lot).
	if res.Parked == nil || res.ParkPoints == nil {
		t.Errorf("Parked=%v ParkPoints=%v, want both set (scenario has a parking lot)", res.Parked, res.ParkPoints)
	}
	t.Logf(
		"native obstacles smoke: laps=%d/%d collided=%t timed_out=%t stuck=%t steps=%d sim_time=%.1fs pass_side_violations=%v",
		res.LapsCompleted, res.TargetLaps, res.Collided, res.TimedOut, res.Stuck, res.Steps, res.SimTimeS,
		res.PassSideViolationSigns,
	)
}
