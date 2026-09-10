package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navigator"
)

// BenchmarkNavigatorStep drives navigator.Navigator.Step (sighted, with the
// fake controllers.HardwareGateway returning a fixed pose + scan) for N
// iterations, reporting ns/op. The fake gateway implements the full
// controllers.HardwareGateway interface (see testutil_test.go).
func BenchmarkNavigatorStep(b *testing.B) {
	gateway := &fakeGateway{}
	gateway.setPose(1.5, 1.0, 0.0)
	gateway.scan = clearScan()
	gateway.haveScan = true

	params := navigator.Params{
		Gateway:           gateway,
		Waypoints:         squareLoop(),
		Direction:         clockwiseDir(),
		Config:            navigator.DefaultConfig(),
		ControllersConfig: controllers.DefaultConfig(),
		Logger:            discardLogger(),
	}
	nav, err := navigator.New(params)
	if err != nil {
		b.Fatalf("New() error = %v", err)
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		// Advance the staged pose a little each tick so the watchdog-driven
		// branches see motion, without depending on published output.
		yaw := math.Mod(float64(i)*0.05, 2*math.Pi)
		gateway.setPose(1.5+0.1*math.Sin(yaw), 1.5+0.1*math.Cos(yaw), yaw)
		nav.Step()
		_, _ = gateway.lastDrive()
	}
}
