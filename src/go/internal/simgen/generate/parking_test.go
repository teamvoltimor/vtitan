package generate_test

import (
	"math"
	"math/rand"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

const spacingTolerance = 1e-9

// The along-travel gap between the two parking blocks is the usable bay
// length the robot must pull into — it must give real clearance beyond
// RobotLength (the dimension that has to fit inside the bay), not merely
// equal it. A prior bug scaled spacing by RobotWidth instead of RobotLength,
// producing a bay exactly as long as the car with zero room to maneuver.
func TestGenerateParkingLotPositions_SpacingClearsRobotLength(t *testing.T) {
	r := generate.NewRandomizer(rand.New(rand.NewSource(1)))

	for _, section := range simconfig.AllSections {
		for trial := range 20 {
			cfg := r.GenerateParkingLotPositions(section)

			var spacing float64
			switch section {
			case simconfig.SectionSouth, simconfig.SectionNorth:
				spacing = cfg.Block2Pos[0] - cfg.Block1Pos[0]
			default: // East, West
				spacing = cfg.Block2Pos[1] - cfg.Block1Pos[1]
			}
			if spacing < 0 {
				spacing = -spacing
			}

			if spacing <= simconfig.RobotLength {
				t.Fatalf(
					"section %s trial %d: parking bay spacing %.3fm does not clear RobotLength %.3fm",
					section, trial, spacing, simconfig.RobotLength,
				)
			}

			wantSpacing := simconfig.ParkingSpacingFactor * simconfig.RobotLength
			if math.Abs(spacing-wantSpacing) > spacingTolerance {
				t.Fatalf(
					"section %s trial %d: spacing = %.3fm, want ParkingSpacingFactor*RobotLength = %.3fm",
					section, trial, spacing, wantSpacing,
				)
			}
		}
	}
}
