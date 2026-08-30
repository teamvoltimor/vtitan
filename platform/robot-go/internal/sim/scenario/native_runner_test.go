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

func writeTempMetadata(t *testing.T) corpus.Scenario {
	t.Helper()
	dir := t.TempDir()
	path := filepath.Join(dir, "scenario_0000_metadata.json")
	if err := os.WriteFile(path, []byte(inlineOpenMetadata), 0o600); err != nil {
		t.Fatalf("writing temp metadata: %v", err)
	}
	return corpus.Scenario{ID: "scenario_0000", MetadataPath: path}
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
