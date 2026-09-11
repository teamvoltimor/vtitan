// Package wallheading_test mirrors tests/unit/test_wall_heading.py's
// PROPERTIES rather than its fixture plumbing: the oracle builds scans from
// simulation.TrackModel and poses from start_conditions.start_pose, neither
// of which is ported yet. trackmodel.TrackWalls raycasts the same Manhattan
// geometry, and the poses here are stated directly instead of derived --
// noted so the difference is not mistaken for a behavioral one.
package wallheading_test

import (
	"fmt"
	"math"
	"math/rand/v2"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/wallheading"
)

const (
	rays          = 360
	noiseSigma    = 0.03
	trackMinCoord = 0.0
	trackMaxCoord = 3.0
	lidarMinRange = 0.045
	lidarMaxRange = 12.0
)

// uniformWalls builds the 1.0 m uniform layout every case here uses;
// corridor-width variation is covered by the localization suite.
func uniformWalls() *trackmodel.TrackWalls {
	const width = 1.0
	widths := map[trackmodel.Section]float64{
		trackmodel.North: width, trackmodel.South: width,
		trackmodel.East: width, trackmodel.West: width,
	}
	geometry := trackmodel.CorridorGeometryFromWidths(widths, trackMaxCoord)
	return trackmodel.NewTrackWalls(geometry, trackMinCoord, trackMaxCoord)
}

// scanAngles mirrors the oracle's linspace(-pi, pi, RAYS) -- a CLOSED
// interval, both endpoints included, which is what that test uses.
func scanAngles() []float64 {
	return navutil.AngleFanClosed(rays)
}

// scan raycasts a sweep at (x, y, yaw), optionally with Gaussian range noise
// from a seeded generator so each case is deterministic.
func scan(
	walls *trackmodel.TrackWalls,
	x, y, yaw, sigma float64,
	seed uint64,
) (ranges, angles []float64) {
	angles = scanAngles()
	ranges = walls.Raycast(x, y, yaw, angles, lidarMinRange, lidarMaxRange)
	if sigma > 0 {
		generator := rand.New(rand.NewPCG(seed, 0x5eed))
		noisy := make([]float64, len(ranges))
		for i, r := range ranges {
			noisy[i] = r + generator.NormFloat64()*sigma
		}
		ranges = noisy
	}
	return ranges, angles
}

func errDeg(estimated, actual float64) float64 {
	return math.Abs(wallheading.HeadingError(estimated, actual) * 180 / math.Pi)
}

// TestRecoversHeadingUnderNoise sweeps headings within one quadrant. The
// walls, not the prior, must decide the answer -- so the prior is handed the
// truth only to select the quadrant.
func TestRecoversHeadingUnderNoise(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	for _, offsetDeg := range []float64{-20, -10, 0, 10, 20} {
		t.Run(formatDeg(offsetDeg), func(t *testing.T) {
			t.Parallel()

			yaw := offsetDeg * math.Pi / 180
			ranges, angles := scan(walls, 1.5, 0.5, yaw, noiseSigma, 1)

			got, ok := wallheading.EstimateYawFromWalls(
				ranges,
				angles,
				yaw,
				wallheading.DefaultConfig(),
			)
			if !ok {
				t.Fatal("no estimate from a structured scan")
			}
			if errDeg(got, yaw) > 5.0 {
				t.Fatalf("estimate = %.2f deg, want %.2f deg within 5", got*180/math.Pi, offsetDeg)
			}
		})
	}
}

// TestWorksFromEveryCorridor checks the Manhattan-world claim holds wherever
// the robot is, not just on one side.
func TestWorksFromEveryCorridor(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	poses := map[string][3]float64{
		"south": {1.5, 0.5, 0.0},
		"east":  {2.5, 1.5, math.Pi / 2},
		"north": {1.5, 2.5, math.Pi},
		"west":  {0.5, 1.5, -math.Pi / 2},
	}

	for name, pose := range poses {
		t.Run(name, func(t *testing.T) {
			t.Parallel()

			x, y, yaw := pose[0], pose[1], pose[2]
			ranges, angles := scan(walls, x, y, yaw, noiseSigma, 2)

			got, ok := wallheading.EstimateYawFromWalls(
				ranges,
				angles,
				yaw,
				wallheading.DefaultConfig(),
			)
			if !ok {
				t.Fatal("no estimate from a structured scan")
			}
			if errDeg(got, yaw) > 5.0 {
				t.Fatalf(
					"estimate = %.2f deg, want %.2f deg within 5",
					got*180/math.Pi,
					yaw*180/math.Pi,
				)
			}
		})
	}
}

// TestDoesNotInheritThePrior is the load-bearing test for this whole module:
// the estimate must be decided by the walls. A prior that is wrong -- but
// still within the same quadrant, so it selects the same candidate -- must
// not drag the answer toward itself.
func TestDoesNotInheritThePrior(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	const trueYaw = 0.0
	ranges, angles := scan(walls, 1.5, 0.5, trueYaw, noiseSigma, 3)

	// 20 degrees of prior error, well inside the +/-45 quadrant.
	priorYaw := 20 * math.Pi / 180

	got, ok := wallheading.EstimateYawFromWalls(
		ranges,
		angles,
		priorYaw,
		wallheading.DefaultConfig(),
	)
	if !ok {
		t.Fatal("no estimate from a structured scan")
	}
	if errDeg(got, trueYaw) > 5.0 {
		t.Fatalf("estimate = %.2f deg, want the TRUE 0 deg within 5 -- it followed the prior",
			got*180/math.Pi)
	}
}

// TestLocksToQuadrantNearestPrior covers the mod-90 ambiguity resolution: the
// same walls admit four answers, and the prior picks which one is meant.
func TestLocksToQuadrantNearestPrior(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	ranges, angles := scan(walls, 1.5, 0.5, 0.0, noiseSigma, 4)

	for quadrant := -1; quadrant <= 2; quadrant++ {
		t.Run(formatDeg(float64(quadrant)*90), func(t *testing.T) {
			t.Parallel()

			priorYaw := float64(quadrant) * math.Pi / 2
			got, ok := wallheading.EstimateYawFromWalls(
				ranges,
				angles,
				priorYaw,
				wallheading.DefaultConfig(),
			)
			if !ok {
				t.Fatal("no estimate from a structured scan")
			}
			if errDeg(got, priorYaw) > 5.0 {
				t.Fatalf("estimate = %.2f deg, want the quadrant at %.0f deg",
					got*180/math.Pi, priorYaw*180/math.Pi)
			}
		})
	}
}

// TestPriorBeyond45DegreesLocksToWrongQuadrant documents the failure mode
// rather than guarding against it: the walls cannot distinguish quadrants, so
// a prior more than 45 degrees off selects the wrong one and the estimate is
// confidently wrong. This is a real limit of the method, not a bug.
func TestPriorBeyond45DegreesLocksToWrongQuadrant(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	const trueYaw = 0.0
	ranges, angles := scan(walls, 1.5, 0.5, trueYaw, noiseSigma, 5)

	priorYaw := 60 * math.Pi / 180

	got, ok := wallheading.EstimateYawFromWalls(
		ranges,
		angles,
		priorYaw,
		wallheading.DefaultConfig(),
	)
	if !ok {
		t.Fatal("no estimate from a structured scan")
	}
	// It locks to 90, not 0 -- so the error against truth is ~90 degrees.
	if errDeg(got, trueYaw) < 45.0 {
		t.Fatalf("estimate = %.2f deg, expected it to lock to the WRONG quadrant", got*180/math.Pi)
	}
}

// TestRefusesToGuess covers the cases where no estimate is better than a bad
// one: low concentration means the returns disagree about where the wall
// runs, which is what a corner or an open side looks like.
func TestRefusesToGuess(t *testing.T) {
	t.Parallel()

	cfg := wallheading.DefaultConfig()

	t.Run("empty scan", func(t *testing.T) {
		t.Parallel()

		if _, ok := wallheading.EstimateYawFromWalls(nil, nil, 0, cfg); ok {
			t.Fatal("produced an estimate from an empty scan")
		}
	})

	t.Run("every ray a no-return", func(t *testing.T) {
		t.Parallel()

		ranges := make([]float64, rays)
		for i := range ranges {
			ranges[i] = lidarMaxRange
		}
		if _, ok := wallheading.EstimateYawFromWalls(ranges, scanAngles(), 0, cfg); ok {
			t.Fatal("produced an estimate where every ray is a no-return")
		}
	})

	t.Run("unstructured noise", func(t *testing.T) {
		t.Parallel()

		generator := rand.New(rand.NewPCG(7, 0x5eed))
		ranges := make([]float64, rays)
		for i := range ranges {
			ranges[i] = 0.5 + generator.Float64()*2.0
		}
		if _, ok := wallheading.EstimateYawFromWalls(ranges, scanAngles(), 0, cfg); ok {
			t.Fatal("produced an estimate from structureless noise")
		}
	})

	t.Run("mismatched ranges and angles", func(t *testing.T) {
		t.Parallel()

		if _, ok := wallheading.EstimateYawFromWalls([]float64{1, 2, 3, 4}, []float64{0, 1}, 0, cfg); ok {
			t.Fatal("produced an estimate from mismatched inputs")
		}
	})
}

// TestStillWorksWithNoNoise guards the clean-scan case, where every segment
// direction agrees exactly and the concentration is at its maximum.
func TestStillWorksWithNoNoise(t *testing.T) {
	t.Parallel()

	walls := uniformWalls()
	const trueYaw = 0.15
	ranges, angles := scan(walls, 1.5, 0.5, trueYaw, 0, 0)

	got, ok := wallheading.EstimateYawFromWalls(
		ranges,
		angles,
		trueYaw,
		wallheading.DefaultConfig(),
	)
	if !ok {
		t.Fatal("no estimate from a clean scan")
	}
	if errDeg(got, trueYaw) > 2.0 {
		t.Fatalf("estimate = %.4f rad, want %.4f within 2 deg", got, trueYaw)
	}
}

// TestHeadingError covers the wrap: the difference between two headings
// either side of pi must be the short way round, not the long one.
func TestHeadingError(t *testing.T) {
	t.Parallel()

	got := wallheading.HeadingError(3.0, -3.0)
	if math.Abs(got-(6.0-2*math.Pi)) > 1e-9 {
		t.Fatalf("HeadingError(3, -3) = %v, want the wrapped short way round", got)
	}
}

func formatDeg(deg float64) string {
	return fmt.Sprintf("%+.0fdeg", deg)
}
