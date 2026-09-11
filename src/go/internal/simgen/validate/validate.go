// Package validate checks generated scenario geometry before writing to disk.
// All checks operate on resolved coordinates (meters, bottom-left origin).
package validate

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

type (
	// Violation is a single failed geometry constraint.
	Violation struct {
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

func (v Violation) Error() string { return fmt.Sprintf("[%s] %s", v.Rule, v.Message) }

// Precomputed minimum-clearance thresholds (meters).
var (
	// Minimum center-to-center spacing between two traffic signs.
	signMinSpacing = simconfig.SignWidth * simconfig.SignSpacingFactor

	// Minimum clearance between a sign center and a parking block center.
	signParkingMinDist = simconfig.ParkingLength/2 + simconfig.SignDepth/2 + simconfig.ValidationClearanceMargin

	// Minimum clearance between robot spawn center and a sign center.
	spawnSignMinDist = simconfig.RobotLength/2 + simconfig.SignDepth/2 + simconfig.ValidationClearanceMargin

	// Minimum clearance between robot spawn center and a parking block center.
	spawnParkingMinDist = simconfig.RobotLength/2 + simconfig.ParkingLength/2 + simconfig.ValidationClearanceMargin

	// Signs must stay this far inside the track boundary.
	signBoundaryMargin = simconfig.SignWidth / 2
)

// ValidateScenario runs all geometry checks and returns any violations found.
// An empty slice means the scenario is valid.
func ValidateScenario(ctx WorldContext) []Violation {
	var violations []Violation
	violations = append(violations, checkSignBounds(ctx.Signs)...)
	violations = append(violations, checkSignOverlap(ctx.Signs)...)
	if ctx.ParkingConfig != nil {
		violations = append(violations, checkParkingBounds(ctx.ParkingConfig)...)
		violations = append(violations, checkSignParkingClearance(ctx.Signs, ctx.ParkingConfig)...)
	}
	violations = append(violations, checkRobotSpawnClearance(ctx.StartingConditions, ctx.Signs, ctx.ParkingConfig)...)
	return violations
}

// checkSignBounds validates that all signs are within the track boundaries.
func checkSignBounds(signs []simconfig.Sign) []Violation {
	lo := simconfig.TrackMinCoord + signBoundaryMargin
	hi := simconfig.TrackMaxCoord - signBoundaryMargin
	var out []Violation
	for i, s := range signs {
		x, y := s.Position[0], s.Position[1]
		if x < lo || x > hi || y < lo || y > hi {
			out = append(out, Violation{
				Rule:    "sign_bounds",
				Message: fmt.Sprintf("sign %d at (%.3f, %.3f) outside track bounds [%.3f, %.3f]", i, x, y, lo, hi),
			})
		}
	}
	return out
}

// checkSignOverlap validates that traffic signs maintain minimum spacing.
func checkSignOverlap(signs []simconfig.Sign) []Violation {
	var out []Violation
	for i := range signs {
		for j := i + 1; j < len(signs); j++ {
			d := dist2d(signs[i].Position, signs[j].Position)
			if d < signMinSpacing {
				out = append(out, Violation{
					Rule:    "sign_overlap",
					Message: fmt.Sprintf("signs %d and %d overlap: dist=%.3f m < min=%.3f m", i, j, d, signMinSpacing),
				})
			}
		}
	}
	return out
}

// checkParkingBounds validates that parking blocks are within track boundaries.
func checkParkingBounds(cfg *simconfig.ParkingConfig) []Violation {
	half := simconfig.ParkingLength / 2
	lo := simconfig.TrackMinCoord + half
	hi := simconfig.TrackMaxCoord - half
	var out []Violation
	for label, pos := range map[string]simconfig.Vec2{simconfig.ParkingBlockIDBlock1: cfg.Block1Pos, simconfig.ParkingBlockIDBlock2: cfg.Block2Pos} {
		x, y := pos[0], pos[1]
		if x < lo || x > hi || y < lo || y > hi {
			out = append(out, Violation{
				Rule:    "parking_bounds",
				Message: fmt.Sprintf("parking %s at (%.3f, %.3f) outside bounds [%.3f, %.3f]", label, x, y, lo, hi),
			})
		}
	}
	return out
}

// checkSignParkingClearance validates minimum distance between signs and parking blocks.
func checkSignParkingClearance(signs []simconfig.Sign, cfg *simconfig.ParkingConfig) []Violation {
	blocks := []struct {
		label string
		pos   simconfig.Vec2
	}{
		{"block1", cfg.Block1Pos},
		{"block2", cfg.Block2Pos},
	}
	var out []Violation
	for si, sign := range signs {
		for _, block := range blocks {
			d := dist2d(sign.Position, block.pos)
			if d < signParkingMinDist {
				out = append(out, Violation{
					Rule: "sign_parking_clearance",
					Message: fmt.Sprintf(
						"sign %d too close to %s: dist=%.3f m < min=%.3f m",
						si,
						block.label,
						d,
						signParkingMinDist,
					),
				})
			}
		}
	}
	return out
}

// checkRobotSpawnClearance validates minimum distance between robot spawn and obstacles.
func checkRobotSpawnClearance(
	sc simconfig.StartingConditions,
	signs []simconfig.Sign,
	cfg *simconfig.ParkingConfig,
) []Violation {
	spawn := sc.Position
	var out []Violation
	for i, sign := range signs {
		d := dist2d(spawn, sign.Position)
		if d < spawnSignMinDist {
			out = append(out, Violation{
				Rule: "spawn_sign_clearance",
				Message: fmt.Sprintf(
					"robot spawn too close to sign %d: dist=%.3f m < min=%.3f m",
					i,
					d,
					spawnSignMinDist,
				),
			})
		}
	}
	if cfg != nil {
		for label, pos := range map[string]simconfig.Vec2{"block1": cfg.Block1Pos, "block2": cfg.Block2Pos} {
			d := dist2d(spawn, pos)
			if d < spawnParkingMinDist {
				out = append(out, Violation{
					Rule: "spawn_parking_clearance",
					Message: fmt.Sprintf(
						"robot spawn too close to parking %s: dist=%.3f m < min=%.3f m",
						label,
						d,
						spawnParkingMinDist,
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
