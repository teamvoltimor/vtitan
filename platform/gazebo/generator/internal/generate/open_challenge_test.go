package generate_test

import (
	"encoding/json"
	"math"
	"os"
	"strings"
	"testing"

	"voldemorbot/gazebo/generator/internal/generate"
	"voldemorbot/gazebo/generator/internal/simconfig"
)

const (
	_yawTolerance = 1e-9
	_nScenarios   = 10
)

var _validYaws = [4]float64{0.0, math.Pi, math.Pi / 2, -math.Pi / 2}

func isValidYaw(yaw float64) bool {
	for _, v := range _validYaws {
		if math.Abs(yaw-v) < _yawTolerance {
			return true
		}
	}
	return false
}

func newOpenGen(t *testing.T, seed int64) *generate.ScenarioGenerator {
	t.Helper()
	gen, err := generate.NewScenarioGenerator(t.TempDir(), simconfig.ScenarioTypeOpen, &seed, nil)
	if err != nil {
		t.Fatalf("NewScenarioGenerator: %v", err)
	}
	return gen
}

func TestOpenChallenge_NoSignsNoParking(t *testing.T) {
	gen := newOpenGen(t, 42)
	for i := range _nScenarios {
		_, meta, err := gen.CreateScenario(i)
		if err != nil {
			t.Errorf("scenario %d: %v", i, err)
			continue
		}
		if meta.NumSigns != 0 {
			t.Errorf("scenario %d: num_signs=%d, want 0", i, meta.NumSigns)
		}
		if len(meta.SignPositions) != 0 {
			t.Errorf("scenario %d: sign_positions len=%d, want 0", i, len(meta.SignPositions))
		}
		if meta.HasParkingLot {
			t.Errorf("scenario %d: has_parking_lot=true, want false", i)
		}
		if meta.ParkingLot != nil {
			t.Errorf("scenario %d: parking_lot not nil", i)
		}
		if meta.ChallengeType != "open" {
			t.Errorf("scenario %d: challenge_type=%q, want open", i, meta.ChallengeType)
		}
		t.Logf("ok | %02d | %s / %s | pos=(%.2f, %.2f) yaw=%.3f",
			i, meta.StartingConditions.Section, meta.StartingConditions.Direction,
			meta.StartingConditions.Position.X, meta.StartingConditions.Position.Y,
			meta.StartingConditions.Yaw)
	}
}

func TestOpenChallenge_CorridorWidthsValid(t *testing.T) {
	gen := newOpenGen(t, 1337)
	validWidthsMM := map[int]bool{600: true, 1000: true}
	sections := []string{"north", "south", "east", "west"}

	for i := range _nScenarios {
		_, meta, err := gen.CreateScenario(i)
		if err != nil {
			t.Errorf("scenario %d: %v", i, err)
			continue
		}
		if len(meta.CorridorWidths) != 4 {
			t.Errorf("scenario %d: expected 4 corridor widths, got %d", i, len(meta.CorridorWidths))
		}
		for _, s := range sections {
			w, ok := meta.CorridorWidths[s]
			if !ok {
				t.Errorf("scenario %d: missing corridor width for %s", i, s)
				continue
			}
			if !validWidthsMM[w.WidthMM] {
				t.Errorf("scenario %d [%s]: width_mm=%d not in {600, 1000}", i, s, w.WidthMM)
			}
			if w.Type != simconfig.WidthTypeNarrow && w.Type != simconfig.WidthTypeWide {
				t.Errorf("scenario %d [%s]: type=%q not narrow|wide", i, s, w.Type)
			}
			expectMM := map[string]int{simconfig.WidthTypeNarrow: 600, simconfig.WidthTypeWide: 1000}
			if got := w.WidthMM; got != expectMM[w.Type] {
				t.Errorf("scenario %d [%s]: type=%s but width_mm=%d (want %d)", i, s, w.Type, got, expectMM[w.Type])
			}
		}
	}
}

func TestOpenChallenge_StartingConditionsValid(t *testing.T) {
	gen := newOpenGen(t, 999)
	for i := range _nScenarios {
		_, meta, err := gen.CreateScenario(i)
		if err != nil {
			t.Errorf("scenario %d: %v", i, err)
			continue
		}
		sc := meta.StartingConditions

		// Position within track
		if sc.Position.X < simconfig.TrackMinCoord || sc.Position.X > simconfig.TrackMaxCoord {
			t.Errorf(
				"scenario %d: x=%.3f outside [%.1f, %.1f]",
				i,
				sc.Position.X,
				simconfig.TrackMinCoord,
				simconfig.TrackMaxCoord,
			)
		}
		if sc.Position.Y < simconfig.TrackMinCoord || sc.Position.Y > simconfig.TrackMaxCoord {
			t.Errorf(
				"scenario %d: y=%.3f outside [%.1f, %.1f]",
				i,
				sc.Position.Y,
				simconfig.TrackMinCoord,
				simconfig.TrackMaxCoord,
			)
		}

		// Yaw is one of the 4 cardinal values
		if !isValidYaw(sc.Yaw) {
			t.Errorf("scenario %d: yaw=%.4f not in {0, ±π/2, π}", i, sc.Yaw)
		}

		// Section and direction are valid
		if _, err := simconfig.ParseSection(strings.ToLower(sc.Section)); err != nil {
			t.Errorf("scenario %d: invalid section %q", i, sc.Section)
		}
		if _, err := simconfig.ParseDirection(sc.Direction); err != nil {
			t.Errorf("scenario %d: invalid direction %q", i, sc.Direction)
		}
	}
}

func TestOpenChallenge_ReproducibleWithSeed(t *testing.T) {
	seed := int64(12345)
	gen1 := newOpenGen(t, seed)
	gen2 := newOpenGen(t, seed)

	for i := range 5 {
		_, m1, err := gen1.CreateScenario(i)
		if err != nil {
			t.Fatalf("gen1 scenario %d: %v", i, err)
		}
		_, m2, err := gen2.CreateScenario(i)
		if err != nil {
			t.Fatalf("gen2 scenario %d: %v", i, err)
		}

		if m1.StartingConditions != m2.StartingConditions {
			t.Errorf("scenario %d: starting conditions differ between runs with same seed", i)
		}
		for _, s := range []string{"north", "south", "east", "west"} {
			if m1.CorridorWidths[s].WidthMM != m2.CorridorWidths[s].WidthMM {
				t.Errorf("scenario %d [%s]: corridor width differs between runs with same seed", i, s)
			}
		}
	}
}

func TestOpenChallenge_WritesFiles(t *testing.T) {
	seed := int64(7)
	gen, err := generate.NewScenarioGenerator(t.TempDir(), simconfig.ScenarioTypeOpen, &seed, nil)
	if err != nil {
		t.Fatalf("NewScenarioGenerator: %v", err)
	}

	worldPath, meta, err := gen.CreateScenario(0)
	if err != nil {
		t.Fatalf("CreateScenario: %v", err)
	}

	info, err := os.Stat(worldPath)
	if err != nil {
		t.Fatalf("SDF file not found at %s: %v", worldPath, err)
	}
	if info.Size() == 0 {
		t.Fatalf("SDF file is empty: %s", worldPath)
	}

	metaPath := strings.Replace(worldPath, ".sdf", "_metadata.json", 1)
	data, err := os.ReadFile(metaPath)
	if err != nil {
		t.Fatalf("metadata not found at %s: %v", metaPath, err)
	}

	var parsed generate.Metadata
	if err := json.Unmarshal(data, &parsed); err != nil {
		t.Fatalf("metadata JSON parse failed: %v", err)
	}
	if parsed.ScenarioID != meta.ScenarioID {
		t.Errorf("scenario_id mismatch: disk=%d memory=%d", parsed.ScenarioID, meta.ScenarioID)
	}
	if parsed.ChallengeType != "open" {
		t.Errorf("disk challenge_type=%q, want open", parsed.ChallengeType)
	}
	t.Logf("SDF: %d bytes | metadata: %d bytes", info.Size(), len(data))
}

func BenchmarkOpenChallenge_CreateScenario(b *testing.B) {
	seed := int64(42)
	gen, err := generate.NewScenarioGenerator(b.TempDir(), simconfig.ScenarioTypeOpen, &seed, nil)
	if err != nil {
		b.Fatalf("NewScenarioGenerator: %v", err)
	}
	b.ResetTimer()
	for i := range b.N {
		if _, _, err := gen.CreateScenario(i); err != nil {
			b.Errorf("scenario %d: %v", i, err)
		}
	}
}
