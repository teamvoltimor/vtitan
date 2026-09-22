//go:build !race

package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
)

// maxDrivingStepAllocs is the allocation ceiling for one Step while the
// navigator is driving. It is a ratchet, not a target: go-future.md 2.5 wants
// the steady-state control loop at zero allocations, and this was 20 when the
// gate was written (2026-09-22). Lower it when a change removes allocations;
// raising it needs a reason in the commit, because every allocation at 20 Hz
// is garbage the collector must eventually pause for.
//
// Excluded under -race, which instruments allocations and changes the count.
const maxDrivingStepAllocs = 20

// allocWindowSteps is how many steps the gate averages over. The fake
// scenario below reaches finished_hold after roughly 250 steps, and a window
// that ran into it would measure the idle post-race state instead, which
// allocates far less. That is what BenchmarkNavigatorStep reports at
// ordinary -benchtime, so its allocs/op is not the driving cost.
const allocWindowSteps = 200

// Deliberately not parallel: AllocsPerRun counts every allocation in the
// process, and Go resumes parallel tests only after the sequential ones end.
//
//nolint:paralleltest // see above
func TestStepAllocationCeilingWhileDriving(t *testing.T) {
	gateway := &fakeGateway{}
	gateway.setPose(1.5, 1.0, 0.0)
	gateway.scan = clearScan()
	gateway.haveScan = true

	nav, err := navigator.New(navigator.Params{
		Gateway:           gateway,
		Waypoints:         squareLoop(),
		Direction:         clockwiseDir(),
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	})
	if err != nil {
		t.Fatalf("New() error = %v", err)
	}

	i := 0
	allocs := testing.AllocsPerRun(allocWindowSteps, func() {
		yaw := math.Mod(float64(i)*0.05, 2*math.Pi)
		gateway.setPose(1.5+0.1*math.Sin(yaw), 1.5+0.1*math.Cos(yaw), yaw)
		nav.Step()
		i++
	})

	if phase := nav.DebugSnapshot().Phase; phase != navigator.PhaseNormalDrive {
		t.Fatalf("window ended in phase %q, want %q: the gate is no longer measuring a driving step",
			phase, navigator.PhaseNormalDrive)
	}
	if allocs > maxDrivingStepAllocs {
		t.Errorf("Step allocates %.2f times per call while driving, ceiling %d", allocs, maxDrivingStepAllocs)
	}
	if allocs < maxDrivingStepAllocs {
		t.Logf("Step allocates %.2f per call, under the ceiling %d: lower maxDrivingStepAllocs", allocs,
			maxDrivingStepAllocs)
	}
}
