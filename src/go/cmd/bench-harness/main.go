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
)

type harnessConfig struct {
	steps      int
	cpuprofile string
	memprofile string
}

func main() {
	cfg := &harnessConfig{}
	root := &cobra.Command{
		Use:   "bench-harness",
		Short: "Run a fixed in-process sim scenario loop and report step timing / profiles",
		RunE: func(_ *cobra.Command, _ []string) error {
			return run(cfg)
		},
	}
	root.Flags().IntVar(&cfg.steps, "steps", 1000, "number of scenario steps to run")
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

	const numRays = 360
	angles := make([]float64, numRays)
	stepAng := 2 * math.Pi / float64(numRays)
	for i := range angles {
		angles[i] = -math.Pi + float64(i)*stepAng
	}

	start := time.Now()
	state := kinematics.AckermannState{X: 1.5, Y: 1.5, Yaw: 0.0, V: 0.5, Steer: 0.1}
	dt := 0.05
	for i := 0; i < steps; i++ {
		_ = tm.RaycastScan(state.X, state.Y, state.Yaw, angles, 0.02, 10.0)
		state = k.Step(state, 0.5, 0.2, dt)
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
	const maxCoord = 3.0
	return trackmodel.CorridorGeometryFromWidths(map[trackmodel.Section]float64{
		trackmodel.North: 0.6,
		trackmodel.South: 0.6,
		trackmodel.East:  0.6,
		trackmodel.West:  0.6,
	}, maxCoord)
}

func benchTrackModel() *collision.TrackModel {
	const maxCoord = 3.0
	return collision.NewTrackModel(collision.NewTrackModelParams{
		Geometry:           benchTrackGeometry(),
		MinCoordM:          0.0,
		MaxCoordM:          maxCoord,
		Obstacles:          nil,
		LidarSeesObstacles: false,
		CollisionMarginM:   0.0,
	})
}

func benchKinematics() *kinematics.AckermannKinematics {
	return kinematics.NewAckermannKinematics(kinematics.DefaultParams())
}
