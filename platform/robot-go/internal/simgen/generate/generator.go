package generate

import (
	"encoding/json"
	"fmt"
	"log/slog"
	"math/rand"
	"os"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/sdf"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/simconfig"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/validate"
)

// ScenarioGenerator orchestrates world randomization, SDF construction, and
// file output for one or more WRO 2026 scenarios.
type ScenarioGenerator struct {
	randomizer    *Randomizer
	strategy      Strategy
	seed          *int64
	challengeType simconfig.ScenarioType
	outputDir     string
}

// NewScenarioGenerator creates a generator for the given challenge type.
// If seed is non-nil, the PRNG is seeded for reproducible output.
func NewScenarioGenerator(
	outputDir string,
	challengeType simconfig.ScenarioType,
	seed *int64,
	strategy Strategy,
) (*ScenarioGenerator, error) {
	if err := os.MkdirAll(outputDir, simconfig.DirPermissions); err != nil {
		return nil, fmt.Errorf("create output dir: %w", err)
	}

	var src rand.Source
	if seed != nil {
		src = rand.NewSource(*seed)
	} else {
		src = rand.NewSource(rand.Int63())
	}
	rng := rand.New(src)
	r := NewRandomizer(rng)

	if strategy == nil {
		strategy = NewFullRandomization(r)
	}

	return &ScenarioGenerator{
		outputDir:     outputDir,
		challengeType: challengeType,
		seed:          seed,
		randomizer:    r,
		strategy:      strategy,
	}, nil
}

// CreateScenario generates one SDF world file and its metadata JSON.
// It retries up to simconfig.MaxScenarioRetries times when geometry validation fails.
func (g *ScenarioGenerator) CreateScenario(idx int) (worldPath string, meta Metadata, err error) {

	corridorWidths := g.resolveCorridorWidths()
	lighting := g.strategy.Lighting()

	// Retry loop for geometry validation
	var (
		sc      simconfig.StartingConditions
		signs   []simconfig.Sign
		parking *simconfig.ParkingConfig
	)
	for attempt := 0; attempt < simconfig.MaxScenarioRetries; attempt++ {
		sc = g.strategy.StartingConditions(corridorWidths)
		signs, parking = g.resolveObstacles(corridorWidths, sc.Section)

		// Resolve the zone before validating, for both challenges, so that
		// checkRobotSpawnClearance runs against the pose the robot ACTUALLY
		// starts from. The Open Challenge zone used to be drawn after the loop
		// and then written over the spawn, leaving the validated position and
		// the emitted one unrelated.
		//
		// Obstacles Challenge starts at the parking bay, not at a point along
		// the starting section: the official round begins with the robot in
		// the lot and requires it to pull out before running laps. The bay is
		// a parallel-park slot ParkingSpacingFactor x RobotLength long (0.45 m
		// for a 0.30 m chassis), and the robot cannot yet start boxed between
		// the two blocks -- so it spawns alongside the bay instead: same
		// along-travel midpoint, offset out to the corridor centreline,
		// already parallel to the outer wall. zoneFromParking computes exactly
		// that pose, and the spawn is its centre.
		//
		// Open Challenge instead keeps the spawn RandomizeStartingConditions
		// already derived from the drawn cell, which sits inside the painted
		// rectangle but not at its centre (see StartingZoneSpawnOffsets).
		//
		// Neither branch consumes RNG, so resolving them inside the retry loop
		// leaves the seeded sequence untouched.
		sc.Zone = GenerateStartingZone(sc.Section, corridorWidths[sc.Section].Width, sc.StartCell, parking)
		if parking != nil {
			sc.Position = simconfig.Vec2{sc.Zone.X, sc.Zone.Y}
		}

		ctx := validate.WorldContext{
			CorridorWidths:     corridorWidths,
			Signs:              signs,
			ParkingConfig:      parking,
			StartingConditions: sc,
		}
		violations := validate.ValidateScenario(ctx)
		if len(violations) == 0 {
			break
		}
		if attempt == simconfig.MaxScenarioRetries-1 {
			msgs := make([]string, len(violations))
			for i, v := range violations {
				msgs[i] = v.Message
			}
			return "", Metadata{}, fmt.Errorf(
				"could not generate valid scenario after %d attempts: %v",
				simconfig.MaxScenarioRetries,
				msgs,
			)
		}
		slog.Warn("scenario geometry invalid, retrying",
			"attempt", attempt+1,
			"max", simconfig.MaxScenarioRetries,
			"violations", violations,
		)
	}

	// Adjust signs that would collide with parking blocks
	if parking != nil {
		signs = adjustSignsForParking(signs, sc.Section)
	}

	// Build SDF world
	root, world := sdf.GenerateBaseWorld()
	sdf.AddSystemPlugins(world)
	sdf.ApplyLighting(world, lighting)
	sdf.AddInteriorWalls(world, corridorWidths)
	sdf.AddTrafficSigns(world, signs)
	if parking != nil {
		sdf.AddParkingLot(world, *parking)
	}
	sdf.AddStartingZone(world, &sc, corridorWidths)
	sdf.AddRobotModel(world, sc)

	// Write SDF
	worldPath, err = g.saveWorld(root, idx)
	if err != nil {
		return "", Metadata{}, err
	}

	meta = BuildMetadata(idx, g.challengeType, corridorWidths, sc, signs, parking, g.seed)
	if err := g.saveMetadata(meta, idx); err != nil {
		return "", Metadata{}, err
	}

	return worldPath, meta, nil
}

// Private helpers

func (g *ScenarioGenerator) resolveCorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	if g.challengeType == simconfig.ScenarioTypeObstacles {
		return FixedCorridorWidths()
	}
	return g.strategy.CorridorWidths()
}

func (g *ScenarioGenerator) resolveObstacles(
	widths map[simconfig.Section]simconfig.CorridorWidth,
	startSection simconfig.Section,
) ([]simconfig.Sign, *simconfig.ParkingConfig) {
	if g.challengeType != simconfig.ScenarioTypeObstacles {
		return nil, nil
	}
	signs := g.randomizer.GenerateSignPositions(widths, &startSection)
	parking := g.randomizer.GenerateParkingLotPositions(startSection)
	return signs, &parking
}

func (g *ScenarioGenerator) saveWorld(root *sdf.Node, idx int) (string, error) {
	name := fmt.Sprintf("%s%04d.sdf", simconfig.ScenarioPrefix, idx)
	path := filepath.Join(g.outputDir, name)
	f, err := os.Create(path)
	if err != nil {
		return "", fmt.Errorf("create world file: %w", err)
	}
	defer f.Close()
	if _, err := root.WriteTo(f); err != nil {
		return "", fmt.Errorf("write world SDF: %w", err)
	}
	return path, nil
}

func (g *ScenarioGenerator) saveMetadata(meta Metadata, idx int) error {
	name := fmt.Sprintf("%s%04d%s", simconfig.ScenarioPrefix, idx, simconfig.MetadataSuffix)
	path := filepath.Join(g.outputDir, name)
	data, err := json.MarshalIndent(meta, "", "  ")
	if err != nil {
		return fmt.Errorf("marshal metadata: %w", err)
	}
	return os.WriteFile(path, data, simconfig.FilePermissions)
}

// adjustSignsForParking moves outer-lane signs in the parking section to the
// inner lane to avoid collisions with the parking blocks.
func adjustSignsForParking(signs []simconfig.Sign, startSection simconfig.Section) []simconfig.Sign {
	outer := simconfig.SignGridWidthOuter
	inner := simconfig.SignGridWidthInner
	tolerance := simconfig.SignAdjustmentTolerance
	trackMax := simconfig.TrackMaxCoord

	result := make([]simconfig.Sign, len(signs))
	for i, s := range signs {
		x, y := s.Position[0], s.Position[1]
		lo, hi := simconfig.TrackCornerMin, simconfig.TrackCornerMax
		switch startSection {
		case simconfig.SectionSouth:
			if lo <= x && x <= hi && abs(y-outer) < tolerance {
				y = inner
			}
		case simconfig.SectionNorth:
			if lo <= x && x <= hi && abs(y-(trackMax-outer)) < tolerance {
				y = trackMax - inner
			}
		case simconfig.SectionEast:
			if lo <= y && y <= hi && abs(x-(trackMax-outer)) < tolerance {
				x = trackMax - inner
			}
		case simconfig.SectionWest:
			if lo <= y && y <= hi && abs(x-outer) < tolerance {
				x = inner
			}
		}
		result[i] = simconfig.Sign{
			Position: simconfig.Vec2{x, y},
			Color:    s.Color,
		}
	}
	return result
}

func abs(x float64) float64 {
	if x < 0 {
		return -x
	}
	return x
}
