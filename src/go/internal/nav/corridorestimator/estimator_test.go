// Package corridorestimator_test mirrors tests/unit/test_corridor_estimator.py.
package corridorestimator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/corridorestimator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

const (
	narrowM = 0.6
	wideM   = 1.0
	rays    = 360
)

// scan builds a sweep whose only meaningful returns are the two
// perpendicular rays, mirroring the oracle's _scan.
//
// The bearings are placed as the simulator places them, which does NOT
// guarantee a ray exactly at +/-pi/2, so the two values are written to
// whichever bearings are nearest -- the same ones the estimator will read.
func scan(leftM, rightM float64) (ranges, angles []float64) {
	angles = make([]float64, rays)
	ranges = make([]float64, rays)
	for i := range angles {
		angles[i] = -math.Pi + 2*math.Pi*float64(i)/float64(rays-1)
		ranges[i] = 3.0
	}

	nearest := func(target float64) int {
		best, bestErr := 0, math.Inf(1)
		for i, a := range angles {
			err := math.Abs(math.Atan2(math.Sin(a-target), math.Cos(a-target)))
			if err < bestErr {
				best, bestErr = i, err
			}
		}
		return best
	}

	ranges[nearest(math.Pi/2)] = leftM
	ranges[nearest(-math.Pi/2)] = rightM
	return ranges, angles
}

func TestMeasureCorridorWidth_SumsThePerpendicularRays(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(0.5, 0.5)
	got, ok := corridorestimator.MeasureCorridorWidth(
		ranges,
		angles,
		0.0,
		corridorestimator.DefaultConfig(),
	)
	if !ok {
		t.Fatal("no measurement from an aligned, plausible scan")
	}
	if math.Abs(got.WidthM-1.0) > 0.02 {
		t.Fatalf("WidthM = %v, want 1.0", got.WidthM)
	}
}

// TestMeasureCorridorWidth_IndependentOfPositionInCorridor is the property
// that makes this need no map: the sum spans wall to wall through the robot
// wherever in the corridor it sits.
func TestMeasureCorridorWidth_IndependentOfPositionInCorridor(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	for _, split := range [][2]float64{{0.5, 0.5}, {0.2, 0.8}, {0.8, 0.2}, {0.1, 0.9}} {
		ranges, angles := scan(split[0], split[1])
		got, ok := corridorestimator.MeasureCorridorWidth(ranges, angles, 0.0, cfg)
		if !ok {
			t.Fatalf("split %v produced no measurement", split)
		}
		if math.Abs(got.WidthM-1.0) > 0.02 {
			t.Fatalf("split %v gave WidthM = %v, want 1.0", split, got.WidthM)
		}
	}
}

func TestMeasureCorridorWidth_WorksOnEveryTrackAxis(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	ranges, angles := scan(0.5, 0.5)
	for _, yaw := range []float64{0, math.Pi / 2, math.Pi, -math.Pi / 2} {
		got, ok := corridorestimator.MeasureCorridorWidth(ranges, angles, yaw, cfg)
		if !ok {
			t.Fatalf("yaw %v produced no measurement", yaw)
		}
		if math.Abs(got.WidthM-1.0) > 0.02 {
			t.Fatalf("yaw %v gave WidthM = %v, want 1.0", yaw, got.WidthM)
		}
	}
}

// TestMeasureCorridorWidth_RejectsMisalignedChassis covers the alignment
// gate: off-axis the side rays cut a longer diagonal than the corridor.
func TestMeasureCorridorWidth_RejectsMisalignedChassis(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(0.5, 0.5)
	yaw := 40 * math.Pi / 180 // beyond the 25 deg tolerance

	if _, ok := corridorestimator.MeasureCorridorWidth(ranges, angles, yaw, corridorestimator.DefaultConfig()); ok {
		t.Fatal("accepted a measurement from a badly misaligned chassis")
	}
}

// TestMeasureCorridorWidth_RejectsEscapedRay covers the plausibility gate: at
// a corner the inward ray misses the inner block and runs off down the next
// corridor, giving a nonsense total.
func TestMeasureCorridorWidth_RejectsEscapedRay(t *testing.T) {
	t.Parallel()

	ranges, angles := scan(3.0, 0.5)

	if _, ok := corridorestimator.MeasureCorridorWidth(ranges, angles, 0.0, corridorestimator.DefaultConfig()); ok {
		t.Fatal("accepted a measurement whose inward ray escaped past the inner block")
	}
}

func TestClassifyWidth(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	tests := map[float64]float64{
		0.55: narrowM,
		0.62: narrowM,
		0.79: narrowM,
		0.80: wideM,
		0.95: wideM,
		1.05: wideM,
	}

	for measured, want := range tests {
		if got := corridorestimator.ClassifyWidth(measured, cfg); math.Abs(got-want) > 1e-9 {
			t.Fatalf("ClassifyWidth(%v) = %v, want %v", measured, got, want)
		}
	}
}

// TestSectionFromHeading mirrors the oracle's table. This is what proves the
// four travel vectors are distinguishable for a fixed direction -- the whole
// basis for attributing a width reading without the position estimate.
func TestSectionFromHeading(t *testing.T) {
	t.Parallel()

	tests := []struct {
		direction  trackmodel.Direction
		headingDeg float64
		want       trackmodel.Section
	}{
		{trackmodel.Clockwise, 180, trackmodel.South},
		{trackmodel.Clockwise, 0, trackmodel.North},
		{trackmodel.Clockwise, -90, trackmodel.East},
		{trackmodel.Clockwise, 90, trackmodel.West},
		{trackmodel.Counterclockwise, 0, trackmodel.South},
		{trackmodel.Counterclockwise, 180, trackmodel.North},
		{trackmodel.Counterclockwise, 90, trackmodel.East},
		{trackmodel.Counterclockwise, -90, trackmodel.West},
	}

	for _, tt := range tests {
		got := corridorestimator.SectionFromHeading(tt.headingDeg*math.Pi/180, tt.direction)
		if got != tt.want {
			t.Fatalf("SectionFromHeading(%v deg, %v) = %v, want %v",
				tt.headingDeg, tt.direction, got, tt.want)
		}
	}
}

// TestSectionFromHeading_ToleratesHeadingError covers the margin: the answer
// must not flip to the next corridor for ordinary heading error.
func TestSectionFromHeading_ToleratesHeadingError(t *testing.T) {
	t.Parallel()

	for _, errorDeg := range []float64{-30, -10, 10, 30} {
		got := corridorestimator.SectionFromHeading(
			errorDeg*math.Pi/180,
			trackmodel.Counterclockwise,
		)
		if got != trackmodel.South {
			t.Fatalf("heading error %v deg gave %v, want South", errorDeg, got)
		}
	}
}

// feed folds `times` identical readings of measuredM into section.
func feed(
	estimator *corridorestimator.WidthEstimator,
	section trackmodel.Section,
	measuredM float64,
	times int,
) []bool {
	changes := make([]bool, 0, times)
	for range times {
		changes = append(changes, estimator.ObserveMeasurement(section, measuredM))
	}
	return changes
}

// newEstimator builds the Open Challenge default: narrow prior, shipped
// tuning, not fixed.
func newEstimator() *corridorestimator.WidthEstimator {
	return corridorestimator.New(narrowM, corridorestimator.DefaultConfig())
}

func TestWidthEstimator_AssumesNarrowBeforeSeeingAnything(t *testing.T) {
	t.Parallel()

	estimator := newEstimator()
	for _, section := range []trackmodel.Section{
		trackmodel.North, trackmodel.South, trackmodel.East, trackmodel.West,
	} {
		if got := estimator.WidthFor(section); math.Abs(got-narrowM) > 1e-9 {
			t.Fatalf("%v starts at %v, want the safe narrow prior %v", section, got, narrowM)
		}
		if estimator.IsObserved(section) {
			t.Fatalf("%v reports observed before any reading", section)
		}
	}
	if estimator.IsComplete() {
		t.Fatal("IsComplete() = true before any reading")
	}
}

// TestWidthEstimator_SeededPriorForRuleBasedRound covers the Obstacles case:
// every corridor is 1.0 m by rule, so starting narrow is known-wrong rather
// than conservative.
func TestWidthEstimator_SeededPriorForRuleBasedRound(t *testing.T) {
	t.Parallel()

	estimator := corridorestimator.New(wideM, corridorestimator.DefaultConfig())
	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-wideM) > 1e-9 {
		t.Fatalf("seeded width = %v, want %v", got, wideM)
	}
}

// TestWidthEstimator_SeededPriorStillOverridden proves seeding changes where
// the estimate STARTS, not whether it measures.
func TestWidthEstimator_SeededPriorStillOverridden(t *testing.T) {
	t.Parallel()

	estimator := corridorestimator.New(wideM, corridorestimator.DefaultConfig())
	feed(estimator, trackmodel.North, narrowM, 20)

	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-narrowM) > 1e-9 {
		t.Fatalf("width = %v, want the measured %v to override the seeded prior", got, narrowM)
	}
}

// TestWidthEstimator_FixedPriorNeverOverridden covers WithFixedWidth: a sign
// hugging one wall can feed a run of falsely-narrow readings with nothing to
// correct them back, and in Obstacles believing narrow re-plans with LESS
// room than exists.
func TestWidthEstimator_FixedPriorNeverOverridden(t *testing.T) {
	t.Parallel()

	estimator := corridorestimator.New(
		wideM,
		corridorestimator.DefaultConfig(),
		corridorestimator.WithFixedWidth(),
	)
	changes := feed(estimator, trackmodel.North, narrowM, 40)

	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-wideM) > 1e-9 {
		t.Fatalf("width = %v, want the fixed %v", got, wideM)
	}
	for _, changed := range changes {
		if changed {
			t.Fatal("a fixed estimator reported a change")
		}
	}
}

func TestWidthEstimator_LearnsAWideCorridor(t *testing.T) {
	t.Parallel()

	estimator := newEstimator()
	feed(estimator, trackmodel.East, wideM, 20)

	if got := estimator.WidthFor(trackmodel.East); math.Abs(got-wideM) > 1e-9 {
		t.Fatalf("width = %v, want %v", got, wideM)
	}
	if !estimator.IsObserved(trackmodel.East) {
		t.Fatal("East not marked observed after 20 agreeing readings")
	}
}

// TestWidthEstimator_ReportsChangeOnce checks the caller is told to replan
// exactly once, not on every tick after the estimate settles.
func TestWidthEstimator_ReportsChangeOnce(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	cfg.MinSamples = 4
	estimator := corridorestimator.New(narrowM, cfg)

	changes := feed(estimator, trackmodel.North, wideM, 8)

	count := 0
	for _, changed := range changes {
		if changed {
			count++
		}
	}
	if count != 1 {
		t.Fatalf("announced %d changes, want exactly 1", count)
	}
}

// TestWidthEstimator_CornerLeakageDoesNotFlipSettledCorridor is the
// regression this class was rewritten for. Corner leakage reads as a wide
// corridor and arrives in RUNS, so a consecutive-agreement rule flips a
// corridor that had already settled correctly. Measured on go_open_0002: a
// truly 0.6 m corridor averaging 0.635 m still peaked at 1.229 m.
func TestWidthEstimator_CornerLeakageDoesNotFlipSettledCorridor(t *testing.T) {
	t.Parallel()

	estimator := newEstimator()
	feed(estimator, trackmodel.North, narrowM, 40)
	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-narrowM) > 1e-9 {
		t.Fatalf("precondition: width = %v, want %v", got, narrowM)
	}

	feed(estimator, trackmodel.North, 1.2, 8)

	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-narrowM) > 1e-9 {
		t.Fatalf("width = %v -- a minority of leaked readings overturned the majority", got)
	}
}

func TestWidthEstimator_IsCompleteOnlyOnceEveryCorridorMeasured(t *testing.T) {
	t.Parallel()

	estimator := newEstimator()
	for _, section := range []trackmodel.Section{trackmodel.North, trackmodel.South, trackmodel.East} {
		feed(estimator, section, wideM, 20)
	}
	if estimator.IsComplete() {
		t.Fatal("IsComplete() = true with West still assumed")
	}

	feed(estimator, trackmodel.West, wideM, 20)
	if !estimator.IsComplete() {
		t.Fatal("IsComplete() = false after every corridor was measured")
	}
}

// TestWidthEstimator_ObserveFoldsAScan covers the scan-taking path, as
// opposed to ObserveMeasurement's already-measured one.
func TestWidthEstimator_ObserveFoldsAScan(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	cfg.MinSamples = 4
	estimator := corridorestimator.New(narrowM, cfg)

	ranges, angles := scan(0.5, 0.5)
	changed := false
	for range 8 {
		if estimator.Observe(trackmodel.North, ranges, angles, 0.0) {
			changed = true
		}
	}

	if !changed {
		t.Fatal("Observe never reported a change from a wide-corridor scan")
	}
	if got := estimator.WidthFor(trackmodel.North); math.Abs(got-wideM) > 1e-9 {
		t.Fatalf("width = %v, want %v", got, wideM)
	}
}

// TestWidthEstimator_RejectedScanChangesNothing covers Observe's early exit:
// a scan the measurement gate rejects must not reach the vote.
func TestWidthEstimator_RejectedScanChangesNothing(t *testing.T) {
	t.Parallel()

	cfg := corridorestimator.DefaultConfig()
	cfg.MinSamples = 1
	estimator := corridorestimator.New(narrowM, cfg)

	// An escaped inward ray: implausible, so never counted.
	ranges, angles := scan(3.0, 0.5)
	for range 10 {
		if estimator.Observe(trackmodel.North, ranges, angles, 0.0) {
			t.Fatal("a rejected scan reported a change")
		}
	}
	if estimator.IsObserved(trackmodel.North) {
		t.Fatal("a rejected scan marked the corridor observed")
	}
}
