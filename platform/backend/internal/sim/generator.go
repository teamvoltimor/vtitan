package sim

import (
	"context"
	"math"
	"math/rand"
	"time"

	"go.uber.org/zap"
	"google.golang.org/protobuf/types/known/timestamppb"

	telemetryv1 "github.com/teamvoldemor/voldemorbot/platform/backend/gen/telemetry/v1"
)

const (
	trackCenterX   = 1.5
	trackCenterY   = 1.5
	orbitPeriod    = 60.0 // frames per full orbit
	orbitBase      = 0.9  // meters
	orbitMod       = 0.2  // amplitude of radius oscillation
	orbitModFreq   = 0.08
	lidarRes       = 360
	lidarBase      = 1.2 // meters
	lidarSineTurb  = 0.08
	lidarRandTurb  = 0.02
	lidarHeight    = 0.04 // Z coordinate of LiDAR plane
	stageSim       = "sim"
	missionNameSim = "sim"
)

type (
	// Store is the write side of the memory store.
	Store interface {
		Write(snap *telemetryv1.RobotSnapshot)
	}

	// Generator produces synthetic RobotSnapshot frames for dev/sim mode.
	// Values are NOT bit-exact reproductions of the Python simulator —
	// the contract is the schema shape, not the numeric sequence.
	Generator struct {
		store Store
		log   *zap.Logger
		rng   *rand.Rand
		frame int
	}
)

// New returns a Generator that publishes synthetic frames to store.
func New(store Store, log *zap.Logger) *Generator {
	return &Generator{
		store: store,
		log:   log,
		rng:   rand.New(rand.NewSource(0)), //nolint:gosec // deterministic sim seed, not crypto
	}
}

// Run publishes a synthetic snapshot at every interval tick until ctx is canceled.
func (g *Generator) Run(ctx context.Context, interval time.Duration) {
	ticker := time.NewTicker(interval)
	defer ticker.Stop()
	g.log.Info("simulator started", zap.Duration("interval", interval))
	for {
		select {
		case <-ctx.Done():
			g.log.Info("simulator stopped")
			return
		case <-ticker.C:
			g.store.Write(g.next())
		}
	}
}

func (g *Generator) next() *telemetryv1.RobotSnapshot {
	t := float64(g.frame)
	g.frame++

	angle := 2 * math.Pi * t / orbitPeriod
	radius := orbitBase + orbitMod*math.Sin(orbitModFreq*t)
	x := trackCenterX + radius*math.Cos(angle)
	y := trackCenterY + radius*math.Sin(angle)

	lidar := make([]*telemetryv1.Position3D, lidarRes)
	for i := range lidar {
		a := 2 * math.Pi * float64(i) / lidarRes
		r := lidarBase + lidarSineTurb*math.Sin(4*a) + lidarRandTurb*g.rng.Float64()
		lidar[i] = &telemetryv1.Position3D{
			X: x + r*math.Cos(a),
			Y: y + r*math.Sin(a),
			Z: lidarHeight,
		}
	}

	orientation := angle
	now := timestamppb.Now()

	return &telemetryv1.RobotSnapshot{
		Timestamp:        now,
		MissionName:      missionNameSim,
		RobotPosition:    &telemetryv1.Position3D{X: x, Y: y, Z: 0},
		RobotOrientation: &orientation,
		LidarPoints:      lidar,
		Metrics: &telemetryv1.TelemetryMetrics{
			Timestamp:      now,
			NodeHealth:     telemetryv1.NodeHealth_NODE_HEALTH_NOMINAL,
			LidarAvailable: true,
			Stage:          stageSim,
			PointsCaptured: lidarRes,
		},
	}
}
