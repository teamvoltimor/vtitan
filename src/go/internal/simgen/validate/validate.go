// Package validate checks generated scenario geometry before writing to disk.
// All checks operate on resolved coordinates (meters, bottom-left origin).
package validate

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// parkingBlock is one parking block's label and placement.
type parkingBlock struct {
	label string
	pos   simconfig.Vec2
}

type (
	// ViolationError is a single failed geometry constraint.
	ViolationError struct {
		Rule    string
		Message string
	}

	// WorldContext holds all resolved scenario parameters needed for validation.
	WorldContext struct {
		CorridorWidths     map[simconfig.Section]simconfig.CorridorWidth
		Signs              []simconfig.Sign
		ParkingConfig      *simconfig.ParkingConfig // nil for open challenge
		StartingConditions simconfig.StartingConditions
	}
)

// clearances holds the precomputed minimum-clearance thresholds (meters) for
// one loaded Track.
type clearances struct {
	signMinSpacing      float64
	signParkingMinDist  float64
	spawnSignMinDist    float64
	spawnParkingMinDist float64
	signBoundaryMargin  float64
}

// computeClearances derives the minimum-clearance thresholds from the track
// and robot geometry.
func computeClearances(track *simconfig.Track, robot *simconfig.Robot) clearances {
	return clearances{
		// Minimum center-to-center spacing between two traffic signs.
		signMinSpacing: track.SignWidth * simconfig.SignSpacingFactor,

		// Minimum clearance between a sign center and a parking block center.
		signParkingMinDist: track.ParkingLength/2 + track.SignDepth/2 + simconfig.ValidationClearanceMargin,

		// Minimum clearance between robot spawn center and a sign center.
		spawnSignMinDist: robot.RobotLength/2 + track.SignDepth/2 + simconfig.ValidationClearanceMargin,

		// Minimum clearance between robot spawn center and a parking block center.
		spawnParkingMinDist: robot.RobotLength/2 + track.ParkingLength/2 + simconfig.ValidationClearanceMargin,

		// Signs must stay this far inside the track boundary.
		signBoundaryMargin: track.SignWidth / 2,
	}
}

func (v ViolationError) Error() string {
	return fmt.Sprintf("[%s] %s", v.Rule, v.Message)
}

// ValidateScenario runs all geometry checks and returns any violations found.
// An empty slice means the scenario is valid.
func ValidateScenario(track *simconfig.Track, robot *simconfig.Robot, ctx WorldContext) []ViolationError {
	c := computeClearances(track, robot)
	var violations []ViolationError
	violations = append(violations, checkSignBounds(track, c, ctx.Signs)...)
	violations = append(violations, checkSignOverlap(c, ctx.Signs)...)
	if ctx.ParkingConfig != nil {
		violations = append(violations, checkParkingBounds(track, ctx.ParkingConfig)...)
		violations = append(violations, checkSignParkingClearance(c, ctx.Signs, ctx.ParkingConfig)...)
	}
	violations = append(
		violations,
		checkRobotSpawnClearance(c, ctx.StartingConditions, ctx.Signs, ctx.ParkingConfig)...)
	return violations
}

// checkSignBounds validates that all signs are within the track boundaries.
func checkSignBounds(track *simconfig.Track, c clearances, signs []simconfig.Sign) []ViolationError {
	lo := track.TrackMinCoord + c.signBoundaryMargin
	hi := track.TrackMaxCoord - c.signBoundaryMargin
	var out []ViolationError
	for i, s := range signs {
		x, y := s.Position[0], s.Position[1]
		if x < lo || x > hi || y < lo || y > hi {
			out = append(out, ViolationError{
				Rule:    "sign_bounds",
				Message: fmt.Sprintf("sign %d at (%.3f, %.3f) outside track bounds [%.3f, %.3f]", i, x, y, lo, hi),
			})
		}
	}
	return out
}

// checkSignOverlap validates that traffic signs maintain minimum spacing.
func checkSignOverlap(c clearances, signs []simconfig.Sign) []ViolationError {
	var out []ViolationError
	for i := range signs {
		for j := i + 1; j < len(signs); j++ {
			d := dist2d(signs[i].Position, signs[j].Position)
			if d < c.signMinSpacing {
				out = append(out, ViolationError{
					Rule: "sign_overlap",
					Message: fmt.Sprintf(
						"signs %d and %d overlap: dist=%.3f m < min=%.3f m",
						i,
						j,
						d,
						c.signMinSpacing,
					),
				})
			}
		}
	}
	return out
}

// checkParkingBounds validates that parking blocks are within track boundaries.
func checkParkingBounds(track *simconfig.Track, cfg *simconfig.ParkingConfig) []ViolationError {
	half := track.ParkingLength / 2
	lo := track.TrackMinCoord + half
	hi := track.TrackMaxCoord - half
	var out []ViolationError
	for label, pos := range map[string]simconfig.Vec2{simconfig.ParkingBlockIDBlock1: cfg.Block1Pos, simconfig.ParkingBlockIDBlock2: cfg.Block2Pos} {
		x, y := pos[0], pos[1]
		if x < lo || x > hi || y < lo || y > hi {
			out = append(out, ViolationError{
				Rule:    "parking_bounds",
				Message: fmt.Sprintf("parking %s at (%.3f, %.3f) outside bounds [%.3f, %.3f]", label, x, y, lo, hi),
			})
		}
	}
	return out
}

// checkSignParkingClearance validates minimum distance between signs and parking blocks.
func checkSignParkingClearance(c clearances, signs []simconfig.Sign, cfg *simconfig.ParkingConfig) []ViolationError {
	blocks := []parkingBlock{
		{"block1", cfg.Block1Pos},
		{"block2", cfg.Block2Pos},
	}
	var out []ViolationError
	for si, sign := range signs {
		for _, block := range blocks {
			d := dist2d(sign.Position, block.pos)
			if d < c.signParkingMinDist {
				out = append(out, ViolationError{
					Rule: "sign_parking_clearance",
					Message: fmt.Sprintf(
						"sign %d too close to %s: dist=%.3f m < min=%.3f m",
						si,
						block.label,
						d,
						c.signParkingMinDist,
					),
				})
			}
		}
	}
	return out
}

// checkRobotSpawnClearance validates minimum distance between robot spawn and obstacles.
func checkRobotSpawnClearance(
	c clearances,
	sc simconfig.StartingConditions,
	signs []simconfig.Sign,
	cfg *simconfig.ParkingConfig,
) []ViolationError {
	spawn := sc.Position
	var out []ViolationError
	for i, sign := range signs {
		d := dist2d(spawn, sign.Position)
		if d < c.spawnSignMinDist {
			out = append(out, ViolationError{
				Rule: "spawn_sign_clearance",
				Message: fmt.Sprintf(
					"robot spawn too close to sign %d: dist=%.3f m < min=%.3f m",
					i,
					d,
					c.spawnSignMinDist,
				),
			})
		}
	}
	if cfg != nil {
		for label, pos := range map[string]simconfig.Vec2{"block1": cfg.Block1Pos, "block2": cfg.Block2Pos} {
			d := dist2d(spawn, pos)
			if d < c.spawnParkingMinDist {
				out = append(out, ViolationError{
					Rule: "spawn_parking_clearance",
					Message: fmt.Sprintf(
						"robot spawn too close to parking %s: dist=%.3f m < min=%.3f m",
						label,
						d,
						c.spawnParkingMinDist,
					),
				})
			}
		}
	}
	return out
}

// dist2d computes the Euclidean distance between two 2D points.
func dist2d(a, b simconfig.Vec2) float64 {
	dx := a[0] - b[0]
	dy := a[1] - b[1]
	return math.Sqrt(dx*dx + dy*dy)
}
