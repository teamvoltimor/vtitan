package collision_test

import (
	"encoding/json"
	"math"
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
)

// goldenAllowedStep is testdata/allowed_step_python.json, written by
// src/python/scripts/sim/gen_allowed_step_golden.py from the Python
// oracle's allowed_step.
type goldenAllowedStep struct {
	Widths    map[string]float64 `json:"widths"`
	Obstacles []struct {
		CX           float64 `json:"cx"`
		CY           float64 `json:"cy"`
		Length       float64 `json:"length"`
		Width        float64 `json:"width"`
		Yaw          float64 `json:"yaw"`
		IsParkingLot bool    `json:"is_parking_lot"`
	} `json:"obstacles"`
	Chassis []float64 `json:"chassis"`
	Cases   []struct {
		Solid     []string  `json:"solid"`
		State     []float64 `json:"state"`
		Candidate []float64 `json:"candidate"`
		Want      []float64 `json:"want"`
		Slide     bool      `json:"slide"`
	} `json:"cases"`
}

// parityTolerance is far below the bisection's own resolution (1/256 of a
// step of at most 8 cm, 0.3 mm), so any disagreement it lets through would
// be rounding, not a different decision.
const parityTolerance = 1e-9

// The Go port decides every golden step exactly as the Python oracle does:
// clear, turn-limited, scaled, slid, or refused (platform plan item 2.11).
func TestAllowedStep_MatchesPythonOracle(t *testing.T) {
	t.Parallel()

	raw, err := os.ReadFile(filepath.Join("testdata", "allowed_step_python.json"))
	if err != nil {
		t.Fatal(err)
	}
	var golden goldenAllowedStep
	if err = json.Unmarshal(raw, &golden); err != nil {
		t.Fatal(err)
	}

	sections := map[string]trackmodel.Section{
		"north": trackmodel.North, "south": trackmodel.South, "east": trackmodel.East, "west": trackmodel.West,
	}
	widths := make(map[trackmodel.Section]float64, len(golden.Widths))
	for name, w := range golden.Widths {
		widths[sections[name]] = w
	}
	specs := make([]collision.ObstacleSpec, 0, len(golden.Obstacles))
	for _, o := range golden.Obstacles {
		specs = append(specs, collision.ObstacleSpec{
			CX: o.CX, CY: o.CY, Length: o.Length, Width: o.Width, Yaw: o.Yaw, IsParkingLot: o.IsParkingLot,
		})
	}
	track := collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:  trackmodel.CorridorGeometryFromWidths(widths, 3.0),
		MinCoordM: 0.0,
		MaxCoordM: 3.0,
		Obstacles: collision.ObstaclesFromSpecs(specs, collision.DefaultAxisAlignTolerance),
	})

	surfaces := map[string]collision.ContactSurface{}
	for _, c := range []collision.ContactSurface{
		collision.SurfaceOuterWall, collision.SurfaceInnerWall, collision.SurfaceObstacle, collision.SurfaceParkingLot,
	} {
		surfaces[c.String()] = c
	}
	pose := func(v []float64) kinematics.AckermannState {
		return kinematics.AckermannState{X: v[0], Y: v[1], Yaw: v[2], V: v[3]}
	}

	var refused, limited int
	for i, c := range golden.Cases {
		solid := collision.NewSurfaceSet()
		for _, name := range c.Solid {
			surface, ok := surfaces[name]
			if !ok {
				t.Fatalf("case %d: unknown surface %q", i, name)
			}
			solid[surface] = struct{}{}
		}
		state, candidate := pose(c.State), pose(c.Candidate)
		got := collision.AllowedStep(track, solid, golden.Chassis[0], golden.Chassis[1], state, candidate, c.Slide)

		switch {
		case c.Want == nil && got != nil:
			t.Errorf("case %d (slide %v): Go moved to %+v, Python refused the step", i, c.Slide, *got)
		case c.Want != nil && got == nil:
			t.Errorf("case %d (slide %v): Go refused the step, Python moved to %v", i, c.Slide, c.Want)
		case c.Want != nil:
			want := pose(c.Want)
			if math.Abs(got.X-want.X) > parityTolerance || math.Abs(got.Y-want.Y) > parityTolerance ||
				math.Abs(got.Yaw-want.Yaw) > parityTolerance || math.Abs(got.V-want.V) > parityTolerance {
				t.Errorf("case %d (slide %v): Go %+v, Python %+v", i, c.Slide, *got, want)
			}
			if want != candidate {
				limited++
			}
		default:
			refused++
		}
	}
	// The fixture must exercise the branches, not only clear steps.
	if refused == 0 || limited == 0 {
		t.Errorf("golden cases: %d refused, %d limited, want both above zero", refused, limited)
	}
}
