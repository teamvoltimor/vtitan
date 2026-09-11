package scenario

import (
	"encoding/json"
	"fmt"
	"log/slog"
	"math"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/visionsim"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
)

// simVisionGateway implements navigator.VisionGateway by emulating sign
// detections from the simulation's TRUE chassis pose each tick, matching
// how ScenarioSimulator wires vision_emulator.emulate_sign_observations
// into the Python gateway. Ground-truth pose only (no believed-pose
// reprojection): the native runner's Localize is unsupported (see
// harness.Config.Localize's doc comment), so there is no separate believed
// pose to diverge from the true one yet.
type simVisionGateway struct {
	gw    simGateway
	signs []signrouter.SignSpec
	cfg   visionsim.Config
}

// signNudgeState accumulates each sign's push-displacement across ticks,
// matching scoring.py's ScenarioSimulator._score_obstacle_contact /
// _sign_push / _prev_contact_xy. One instance per run.
type signNudgeState struct {
	push         map[int]float64
	prevX, prevY float64
}

func (v *simVisionGateway) GetVisionDetections() ([]signrouter.TrafficSignObservation, bool) {
	st := v.gw.State()
	obs := visionsim.EmulateSignObservations(v.signs, st.X, st.Y, st.Yaw, v.cfg, nil)
	return obs, len(obs) > 0
}

// signsFromMetadata builds the ground-truth SignSpec list for an Obstacles
// Challenge scenario, matching the sign half of track_model.py's
// obstacles_from_metadata (the parking-block half is deliberately not
// ported here -- see collision.ObstacleBox's is_parking_lot gap, tracked
// separately). Empty for Open Challenge metadata (no sign_positions key),
// exactly as Python's obstacles_from_metadata returns an empty list for it.
func signsFromMetadata(meta generate.Metadata) []signrouter.SignSpec {
	if len(meta.SignPositions) == 0 {
		return nil
	}
	signs := make([]signrouter.SignSpec, len(meta.SignPositions))
	for i, s := range meta.SignPositions {
		color := signrouter.SignColorGreen
		if s.Color == "red" {
			color = signrouter.SignColorRed
		}
		signs[i] = signrouter.SignSpec{X: s.X, Y: s.Y, Color: color}
	}
	return signs
}

// parkBlocksFromMetadata builds the two parking-lot marker fins as
// collision.ObstacleSpec, matching the parking half of obstacles_from_metadata
// (track_model.py) -- the sign half lives in signsFromMetadata. Empty when
// the scenario has no parking lot.
func parkBlocksFromMetadata(meta generate.Metadata) []collision.ObstacleSpec {
	if meta.ParkingLot == nil {
		return nil
	}
	lot := meta.ParkingLot
	return []collision.ObstacleSpec{
		{
			CX: lot.Block1Position.X, CY: lot.Block1Position.Y,
			Length: parking.DefaultParkingLotLengthM, Width: parking.DefaultParkingLotWidthM,
			Yaw: lot.Block1Yaw, IsParkingLot: true,
		},
		{
			CX: lot.Block2Position.X, CY: lot.Block2Position.Y,
			Length: parking.DefaultParkingLotLengthM, Width: parking.DefaultParkingLotWidthM,
			Yaw: lot.Block2Yaw, IsParkingLot: true,
		},
	}
}

// parkControllerFromMetadata builds a parking.ParkController from scenario
// metadata, matching park_controller_from_metadata. nil when the scenario
// has no parking lot.
func parkControllerFromMetadata(
	meta generate.Metadata, section trackmodel.Section, direction trackmodel.Direction, cfg parking.Config,
) *parking.ParkController {
	if meta.ParkingLot == nil {
		return nil
	}
	lot := &parking.ParkingLot{
		Block1: parking.BlockPosition{X: meta.ParkingLot.Block1Position.X, Y: meta.ParkingLot.Block1Position.Y},
		Block2: parking.BlockPosition{X: meta.ParkingLot.Block2Position.X, Y: meta.ParkingLot.Block2Position.Y},
	}
	return parking.ParkControllerFromMetadata(lot, section, direction, cfg)
}

// passSideCheck re-arms the scorer at a lap boundary and returns any
// wrong-side violations committed this tick, mirroring the Python run loop's
// `if violation_signs is not None: break`.
func passSideCheck(
	nav *navigator.Navigator, st kinematics.AckermannState, scorer *passSideScorer, prevLaps *int,
) []int {
	if lapsNow := nav.LapsCompleted(); lapsNow != *prevLaps {
		scorer.resetForNewLap()
		*prevLaps = lapsNow
	}
	return scorer.check(st.X, st.Y, st.Yaw)
}

// roundSettled reports whether the run is over: the lap target is reached and,
// when a ParkController is attached, the parking maneuver has also finished
// (parked cleanly or gave up), matching the Python run loop's
// `laps_completed >= num_laps and (pc is None or pc.is_done)`.
//
// The runner must read AttemptAfterFinalLap, not just IsDone: with the pursuit
// deferred the controller exists and never completes, so an IsDone-only break
// keeps stepping a robot already at rest and charges the whole budget to
// sim_time_s -- the exact number in-time is measured against.
func roundSettled(nav *navigator.Navigator, targetLaps int) bool {
	if nav.LapsCompleted() < targetLaps {
		return false
	}
	pc := nav.ParkController()
	return pc == nil || !pc.AttemptAfterFinalLap() || pc.IsDone()
}

func resTimedOut(steps, maxSteps, laps, target int) bool {
	return steps >= maxSteps && laps < target
}

func avgSpeed(distanceM float64, steps int, dt float64) float64 {
	if steps <= 0 || dt <= 0 {
		return 0
	}
	return distanceM / (float64(steps) * dt)
}

func orZero(v float64) float64 {
	if math.IsInf(v, 0) || math.IsNaN(v) {
		return 0
	}
	return v
}

// newSignNudgeState seeds the reference point at the run's start pose,
// matching _prev_contact_xy's __init__ assignment.
func newSignNudgeState(x, y float64) *signNudgeState {
	return &signNudgeState{prevX: x, prevY: y, push: make(map[int]float64)}
}

// score downgrades a legal pillar nudge (WRO 9.20) to a non-collision surface,
// matching _score_obstacle_contact. Fin/wall surfaces pass through untouched
// -- only SurfaceObstacle (traffic signs; no parking fin ever reaches this
// runner yet, see signsFromMetadata) gets the leniency.
//
// The reference point advances EVERY call, not only while touching: updating
// it only during contact would make the accumulated displacement the
// distance since the last touch, so a sign brushed twice a meter apart would
// accumulate that whole meter as if it had been pushed through it.
func (s *signNudgeState) score(
	track *collision.TrackModel, surface collision.ContactSurface, x, y, yaw, length, width float64,
) collision.ContactSurface {
	dx, dy := x-s.prevX, y-s.prevY
	s.prevX, s.prevY = x, y
	if surface != collision.SurfaceObstacle {
		return surface
	}
	for index := range track.ObstacleDisplacements(x, y, yaw, length, width) {
		// Only the component of travel pointing AT the sign moves it: a
		// chassis sliding past a sign it is brushing covers distance
		// without pushing it anywhere.
		center, ok := track.ObstacleCenter(index)
		if !ok {
			continue
		}
		toX, toY := center.X-x, center.Y-y
		norm := math.Hypot(toX, toY)
		if norm <= 0.0 {
			continue
		}
		push := (dx*toX + dy*toY) / norm
		if push > 0.0 {
			s.push[index] += push
		}
	}
	for _, push := range s.push {
		if push > maxLegalSignDisplacementM {
			return surface
		}
	}
	// Touched, but still inside its placement circle: not a collision.
	return collision.SurfaceNone
}

// loadMetadata reads and parses a *_metadata.json file into generate.Metadata
// -- the SAME struct cmd/simgen writes (internal/simgen/generate), not a
// separately maintained mirror of its schema. The two used to be independent,
// hand-kept-in-sync definitions (one per Go module); unified once simgen
// joined this module, so a schema change in one can no longer silently drift
// from the other.
func loadMetadata(path string) (generate.Metadata, error) {
	raw, err := os.ReadFile(filepath.Clean(path))
	if err != nil {
		return generate.Metadata{}, fmt.Errorf("reading %s: %w", path, err)
	}
	var meta generate.Metadata
	if unmarshalErr := json.Unmarshal(raw, &meta); unmarshalErr != nil {
		return generate.Metadata{}, fmt.Errorf("parsing %s: %w", path, unmarshalErr)
	}
	return meta, nil
}

// sightedCenterBiasM is the planning bias for a sighted round: nil on Open,
// which takes waypoints.toml's narrow/wide split, and the uniform Obstacles
// override otherwise. Mirrors blindCenterBiasM, keyed off the same fact
// (does this scenario carry signs) rather than off the metadata's
// challenge_type string, so a mislabeled fixture cannot plan one challenge
// with the other's bias.
func sightedCenterBiasM(meta generate.Metadata, wpCfg waypoints.Config) *float64 {
	return blindCenterBiasM(len(meta.SignPositions) > 0, wpCfg)
}

// defaultLaps returns the Open Challenge default lap count.
func defaultLaps(_ generate.Metadata) int {
	return navigator.DefaultOpenChallengeLaps
}

// roundTimeLimitSFor reads competition_specs.toml's round_time_limit_s, or
// falls back to the shipped default when there is no config root or the file
// will not load. Scoring a run against a hardcoded limit is the same class of
// bug as reading a hardcoded sensor spec: the rule book is a file.
func roundTimeLimitSFor(logger *slog.Logger, configRoot string) float64 {
	fallback, ok := profile.CompetitionDefaults()["round_time_limit_s"].(float64)
	if !ok {
		logger.Warn("native runner: competition defaults missing round_time_limit_s, using fallback")
		fallback = defaultRoundTimeLimitS
	}
	if configRoot == "" {
		return fallback
	}
	path := filepath.Join(configRoot, profile.DefaultCompetitionTOMLPath)
	cc, err := profile.LoadWithDefaults[profile.CompetitionConfig](
		path, nil, profile.CompetitionDefaults(),
	)
	if err != nil {
		logger.Warn("native runner: loading competition_specs.toml, falling back to default",
			"config_root", configRoot, "error", err)
		return fallback
	}
	return cc.RoundTimeLimitS
}
