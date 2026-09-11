package scenario

import (
	"fmt"
	"io"
	"log/slog"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/widthbelief"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
)

// blindNarrowWidthM is the corridor width a blind OPEN round assumes before
// it has measured anything: the narrow (fail-safe) end of the 60/100 cm pair
// the rules allow. Believing narrow and finding wide leaves the robot with
// room; the reverse puts the planned line inside a wall.
//
// Mirrors CorridorDimensions.NARROW, which the Python simulator passes as
// CorridorWidthEstimator's assumed_width for Open.
const blindNarrowWidthM = 0.6

// obstaclesCorridorWidthM is what a blind OBSTACLES round assumes, and keeps:
// every corridor is 1.0 m by rule, so this is prior KNOWLEDGE, not a guess.
// The estimator is fixed there for that reason -- a sign hugging a wall can
// otherwise feed the voting a run of falsely-narrow readings with nothing to
// correct it back.
//
// Mirrors CorridorDimensions.OBSTACLES_WIDTH.
const obstaclesCorridorWidthM = 1.0

// blindSetup is everything Run needs to hand the navigator a blind round
// instead of a sighted one.
type blindSetup struct {
	// Waypoints is the initial path, planned from the PRIOR layout and the
	// ASSUMED start rather than the scenario's truth.
	Waypoints []trackmodel.Waypoint
	// Layout is the per-tick belief loop that corrects that path as corridors
	// are measured. Nil is impossible here (blindSetup is only built when
	// blind), but Layout.Update tolerates nil anyway.
	Layout *widthbelief.Layout
}

// newBlindSetup builds the blind round's initial path and belief loop.
//
// The initial path is planned with the REAL planner
// (waypoints.CalculateWaypoints), the same one buildScenario now uses for a
// sighted round: every subsequent replan comes from that planner via
// widthbelief.Layout, and a round that switched planners on its first width
// update would report a path discontinuity that no belief change caused.
//
// The believed START is the assumed pose, not the scenario's: a path is built
// from where the robot thinks it is, and blind is exactly the case where that
// differs. Planning from the true start while the robot navigates in the
// believed frame hands it a route to a place it does not think it is.
//
// isObstacles selects the prior and switches off both the estimator's voting
// and the deferral gate, matching the Python simulator's own construction.
func newBlindSetup(
	base waypoints.PlannerInput,
	direction trackmodel.Direction,
	isObstacles bool,
	cfg harness.Config,
	waypointCfg waypoints.Config,
	startCfg startconditions.Config,
	centerBiasM *float64,
) (blindSetup, error) {
	priorWidthM := blindNarrowWidthM
	if isObstacles {
		priorWidthM = obstaclesCorridorWidthM
	}
	prior := map[trackmodel.Section]float64{
		trackmodel.North: priorWidthM,
		trackmodel.South: priorWidthM,
		trackmodel.East:  priorWidthM,
		trackmodel.West:  priorWidthM,
	}
	priorGeometry := trackmodel.CorridorGeometryFromWidths(prior, cfg.TrackMaxCoordM)

	// The assumed START takes a different bias from the PLAN, and only on
	// Obstacles: its scenarios are calibrated around the pre-split WIDE
	// magnitude, so its assumed start does not sit on its own planned line.
	// A pre-existing inconsistency in the Python original, reproduced rather
	// than fixed in passing -- the Obstacles bias is a separately swept value.
	var assumedBiasM *float64
	if isObstacles {
		bias := waypointCfg.WideCenterBiasM
		assumedBiasM = &bias
	}
	assumed, ok := startconditions.AssumedStartConditions(
		direction,
		prior,
		startconditions.CanonicalSection,
		startCfg,
		assumedBiasM,
	)
	if !ok {
		return blindSetup{}, fmt.Errorf(
			"blind setup: no assumed start pose for %v from the canonical section", direction,
		)
	}

	// A blind round has measured nothing yet, so every corridor is planned
	// with the unconfirmed inner bias.
	planned := base
	planned.Geometry = priorGeometry
	planned.Starting = base.Starting.ReplannedAt(
		&direction,
		assumed.Section,
		trackmodel.Waypoint{X: assumed.X, Y: assumed.Y},
		assumed.Yaw,
	)
	path, err := waypoints.CalculateWaypoints(
		planned, 1, waypointCfg, centerBiasM, waypoints.AllUnconfirmed(),
	)
	if err != nil {
		return blindSetup{}, fmt.Errorf("blind setup: planning the prior layout: %w", err)
	}

	estimatorOpts := []corridorestimator.Option{}
	if isObstacles {
		estimatorOpts = append(estimatorOpts, corridorestimator.WithFixedWidth())
	}
	layout := widthbelief.NewLayout(widthbelief.Params{
		Logger: discardingLogger(),
		Estimator: corridorestimator.New(
			priorWidthM, corridorestimator.DefaultConfig(), estimatorOpts...,
		),
		// Deferral is OPEN-ONLY. On Obstacles the estimator is fixed and the
		// bias is an explicit override, so the gate's confirmed-ness trigger
		// would rebuild a byte-identical path and re-seek the waypoint index
		// for nothing.
		Defer:       !isObstacles && waypointCfg.DeferCurrentCorridorReplan,
		Base:        base,
		Config:      waypointCfg,
		CenterBiasM: centerBiasM,
		MaxCoordM:   cfg.TrackMaxCoordM,
	})

	return blindSetup{Waypoints: path, Layout: layout}, nil
}

// discardingLogger is the logger the belief loop writes to inside a corpus
// sweep. A sweep runs hundreds of scenarios concurrently and a per-replan
// info line from each would bury the report; the belief is observable through
// Layout.Believed and the Result either way.
func discardingLogger() *slog.Logger {
	return slog.New(slog.NewTextHandler(io.Discard, nil))
}

// obstaclesCenterBiasM mirrors waypoints.OBSTACLES_CENTER_BIAS_M: the
// uniform centreline bias an Obstacles round plans with, overriding the
// narrow/wide split entirely (its corridors are all 1.0 m by rule, so there
// is no narrow case for the split to describe).
//
// Measured HIGHER than the geometry argues for, compensating for the
// tracker's outward drift rather than describing a racing line.
const obstaclesCenterBiasM = 0.15

// blindCenterBiasM is the planning bias for a blind round: nil on Open,
// which takes the narrow/wide split, and the Obstacles override otherwise.
func blindCenterBiasM(isObstacles bool) *float64 {
	if !isObstacles {
		return nil
	}
	bias := obstaclesCenterBiasM
	return &bias
}

// plannerBaseFor builds the PlannerInput a blind round replans from. Its
// Geometry and Starting are replaced on every plan (by the prior at setup,
// by the believed layout thereafter), so only the two track-level facts
// matter here.
func plannerBaseFor(_ generate.Metadata, cfg harness.Config) waypoints.PlannerInput {
	return waypoints.PlannerInput{
		MaxCoordM:     cfg.TrackMaxCoordM,
		ChassisWidthM: cfg.ChassisWidthM,
	}
}
