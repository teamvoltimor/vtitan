// Command bench-harness runs a fixed Obstacles-style sim scenario loop in
// process and reports per-step timing. With --cpuprofile / --memprofile it
// captures runtime/pprof profiles around the loop. It does not edit or reuse
// cmd/sim-runner; it is a standalone profiling harness (see
// docs/internal/plans/2026-08-30-python-go-comparison.md §4).
package main

import (
	"fmt"
	"math"
	"os"
	"runtime/pprof"
	"time"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

type harnessConfig struct {
	steps      int
	cpuprofile string
	memprofile string
}

// Benchmark loop constants: a synthetic ray count plus the fixed start pose,
// command inputs, and corridor width the harness exercises.
const (
	benchNumRays        = 360
	benchDefaultSteps   = 1000
	benchStartXM        = 1.5
	benchStartYM        = 1.5
	benchStartV         = 0.5
	benchRayMinRangeM   = 0.02
	benchRayMaxRangeM   = 10.0
	benchSpeedMPS       = 0.5
	benchSteerRad       = 0.2
	benchStartSteerRad  = 0.1
	benchDT             = 0.05
	benchCorridorWidthM = 0.6
)

func main() {
	cfg := &harnessConfig{}
	root := &cobra.Command{
		Use:   "bench-harness",
		Short: "Run a fixed in-process sim scenario loop and report step timing / profiles",
		RunE: func(_ *cobra.Command, _ []string) error {
			return run(cfg)
		},
	}
	root.Flags().IntVar(&cfg.steps, "steps", benchDefaultSteps, "number of scenario steps to run")
	root.Flags().StringVar(&cfg.cpuprofile, "cpuprofile", "", "write a CPU profile to this path")
	root.Flags().StringVar(&cfg.memprofile, "memprofile", "", "write a memory profile to this path")

	if err := root.Execute(); err != nil {
		fmt.Fprintln(os.Stderr, err)
		os.Exit(1)
	}
}

// run builds the synthetic track + kinematics, optionally starts pprof, runs
// the fixed loop, and reports timing.
func run(cfg *harnessConfig) error {
	if cfg.cpuprofile != "" {
		f, err := os.Create(cfg.cpuprofile)
		if err != nil {
			return fmt.Errorf("creating cpu profile: %w", err)
		}
		defer f.Close()
		if perr := pprof.StartCPUProfile(f); perr != nil {
			return fmt.Errorf("starting cpu profile: %w", perr)
		}
		defer pprof.StopCPUProfile()
	}

	steps := cfg.steps
	if steps <= 0 {
		steps = 1
	}

	tm := benchTrackModel()
	k := benchKinematics()

	angles := make([]float64, benchNumRays)
	stepAng := 2 * math.Pi / float64(benchNumRays)
	for i := range angles {
		angles[i] = -math.Pi + float64(i)*stepAng
	}

	start := time.Now()
	state := kinematics.AckermannState{
		X: benchStartXM, Y: benchStartYM, Yaw: 0.0, V: benchStartV, Steer: benchStartSteerRad,
	}
	dt := benchDT
	for i := 0; i < steps; i++ {
		_ = tm.RaycastScan(state.X, state.Y, state.Yaw, angles, benchRayMinRangeM, benchRayMaxRangeM)
		state = k.Step(state, benchSpeedMPS, benchSteerRad, dt)
	}
	elapsed := time.Since(start)

	fmt.Printf("ran %d steps in %v (%.1f ns/op, %.0f steps/s)\n",
		steps, elapsed, float64(elapsed.Nanoseconds())/float64(steps),
		float64(steps)/elapsed.Seconds())

	if cfg.memprofile != "" {
		f, err := os.Create(cfg.memprofile)
		if err != nil {
			return fmt.Errorf("creating mem profile: %w", err)
		}
		defer f.Close()
		if perr := pprof.WriteHeapProfile(f); perr != nil {
			return fmt.Errorf("writing mem profile: %w", perr)
		}
		fmt.Printf("wrote memory profile to %s\n", cfg.memprofile)
	}
	if cfg.cpuprofile != "" {
		fmt.Printf("wrote cpu profile to %s\n", cfg.cpuprofile)
	}
	return nil
}

func benchTrackGeometry() trackmodel.CorridorGeometry {
	return trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: benchCorridorWidthM,
		trackmodel.South: benchCorridorWidthM,
		trackmodel.East:  benchCorridorWidthM,
		trackmodel.West:  benchCorridorWidthM,
	}, simconfig.TrackMaxCoord)
}

func benchTrackModel() *collision.TrackModel {
	return collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           benchTrackGeometry(),
		MinCoordM:          0.0,
		MaxCoordM:          simconfig.TrackMaxCoord,
		Obstacles:          nil,
		LidarSeesObstacles: false,
		CollisionMarginM:   0.0,
	})
}

func benchKinematics() *kinematics.AckermannKinematics {
	return kinematics.NewAckermannKinematics(kinematics.DefaultParams())
}
