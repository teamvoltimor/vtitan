package parking_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// lotCentreX/wallOffsetY match the Python fixture: a SOUTH lot with fins
// BLOCK_SPACING_FACTOR x chassis-LENGTH apart, standing off the wall.
const lotCentreX = 1.5

func scoringZone() parking.ParkZone {
	halfSpan := parking.DefaultParkingLotBlockSpacingFactor * parking.DefaultChassisLengthM / 2
	return parking.BuildZone(
		parking.BlockPosition{X: lotCentreX - halfSpan, Y: parking.DefaultParkingLotWallOffsetM},
		parking.BlockPosition{X: lotCentreX + halfSpan, Y: parking.DefaultParkingLotWallOffsetM},
		trackmodel.South,
		trackmodel.Counterclockwise,
		parking.DefaultParkingLotSpecs,
		parking.DefaultTrackDimensions,
	)
}

func alongCentre(z parking.ParkZone) float64 {
	lo, hi := z.BoundsAlong()
	return (lo + hi) / 2
}

func TestScorePark_ParallelAndContainedScoresFull(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	depthLo, depthHi := z.BoundsDepth()

	score := parking.ScorePark(alongCentre(z), (depthLo+depthHi)/2, z.TargetYaw, z, cfg)

	if score.Points != parking.FullParkPoints {
		t.Errorf("Points = %v, want %v", score.Points, parking.FullParkPoints)
	}
	if !score.Contained || !score.Parallel || score.Touched {
		t.Errorf("score = %+v, want contained+parallel, not touched", score)
	}
}

func TestScorePark_PerpendicularNoseInScoresPartial(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	depthLo, _ := z.BoundsDepth()
	const standoff = 0.02
	noseDepth := depthLo + standoff

	score := parking.ScorePark(
		alongCentre(z), noseDepth+parking.DefaultChassisLengthM/2, math.Pi/2, z, cfg,
	)

	if score.Points != parking.PartialParkPoints {
		t.Errorf("Points = %v, want %v", score.Points, parking.PartialParkPoints)
	}
	if !score.Overlaps || score.Contained || score.Touched {
		t.Errorf("score = %+v, want overlaps, not contained, not touched", score)
	}
}

func TestScorePark_PerpendicularEntryHasRoomToSpare(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	alongLo, alongHi := z.BoundsAlong()
	depthLo, _ := z.BoundsDepth()
	centreDepth := depthLo + 0.02 + parking.DefaultChassisLengthM/2
	slack := (alongHi - alongLo - parking.DefaultChassisWidthM) / 2

	for _, offset := range []float64{-slack + 0.005, 0.0, slack - 0.005} {
		score := parking.ScorePark(alongCentre(z)+offset, centreDepth, math.Pi/2, z, cfg)
		if score.Points != parking.PartialParkPoints {
			t.Errorf("offset %v: Points = %v, want %v", offset, score.Points, parking.PartialParkPoints)
		}
		if score.Touched {
			t.Errorf("offset %v: touched, want clear", offset)
		}
	}
}

func TestScorePark_TouchingAFinVoidsAllPoints(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	_, alongHi := z.BoundsAlong()
	depthLo, _ := z.BoundsDepth()
	beyondFin := alongHi + parking.DefaultChassisWidthM/2 - 0.01

	score := parking.ScorePark(
		beyondFin, depthLo+0.02+parking.DefaultChassisLengthM/2, math.Pi/2, z, cfg,
	)

	if !score.Touched {
		t.Errorf("score = %+v, want touched", score)
	}
	if score.Points != 0 {
		t.Errorf("Points = %v, want 0", score.Points)
	}
}

func TestScorePark_OutInTheCorridorScoresNothing(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	_, depthHi := z.BoundsDepth()

	score := parking.ScorePark(
		alongCentre(z), depthHi+parking.DefaultChassisLengthM, z.TargetYaw, z, cfg,
	)

	if score.Points != 0 {
		t.Errorf("Points = %v, want 0", score.Points)
	}
	if score.Overlaps || score.Touched {
		t.Errorf("score = %+v, want no overlap, no touch", score)
	}
}

func TestScorePark_ParallelTestAcceptsEitherHeading(t *testing.T) {
	t.Parallel()
	z := scoringZone()
	cfg := parking.DefaultConfig()
	depthLo, depthHi := z.BoundsDepth()
	reversedYaw := z.TargetYaw + math.Pi

	score := parking.ScorePark(alongCentre(z), (depthLo+depthHi)/2, reversedYaw, z, cfg)

	if !score.Parallel {
		t.Errorf("score = %+v, want parallel", score)
	}
	if score.Points != parking.FullParkPoints {
		t.Errorf("Points = %v, want %v", score.Points, parking.FullParkPoints)
	}
}
