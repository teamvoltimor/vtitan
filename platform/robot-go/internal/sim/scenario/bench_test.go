package scenario

import (
	"fmt"
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/kinematics"
)

// benchTrackGeometry is a small synthetic WRO-style layout with a square
// outer boundary and a centered inner block, so the benchmark depends on no
// external corpus file.
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
	return kinematics.NewAckermannKinematics(kinematics.Params{
		WheelbaseM:          0.30,
		MaxSteerRad:         0.50,
		MaxSteerRateRadPerS: 3.0,
		MaxAccelMPS2:        1.5,
		MaxSpeedMPS:         1.0,
		RearSteerRatio:      1.0,
		SpeedTauS:           0.1,
		YawGain:             0.55,
		Substeps:            5,
	})
}

// BenchmarkScenarioStep builds a collision.TrackModel + kinematics and runs N
// control steps (raycast scan + kinematic advance) against the fixed synthetic
// track, reporting ns/op.
func BenchmarkScenarioStep(b *testing.B) {
	const seedX, seedY, seedYaw = 1.5, 1.5, 0.0
	const dt = 0.05

	tm := benchTrackModel()
	k := benchKinematics()

	const numRays = 360
	angles := make([]float64, numRays)
	step := 2 * math.Pi / float64(numRays)
	for i := range angles {
		angles[i] = -math.Pi + float64(i)*step
	}

	b.ResetTimer()
	for i := 0; i < b.N; i++ {
		state := kinematics.AckermannState{X: seedX, Y: seedY, Yaw: seedYaw, V: 0.5, Steer: 0.1}
		_ = tm.RaycastScan(state.X, state.Y, state.Yaw, angles, 0.02, 10.0)
		state = k.Step(state, 0.5, 0.2, dt)
		_ = state
	}
	_ = fmt.Sprint
}
