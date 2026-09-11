package startconditions_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

func uniformWidths(widthM float64) map[trackmodel.Section]float64 {
	return map[trackmodel.Section]float64{
		trackmodel.North: widthM, trackmodel.South: widthM,
		trackmodel.East: widthM, trackmodel.West: widthM,
	}
}

// TestStartPose_Centerline checks each section's spawn pose sits at the
// middle of the mat's side, offset half a corridor width off the outer
// wall plus the configured bias, and yaw aligned with travel.
func TestStartPose_Centerline(t *testing.T) {
	t.Parallel()

	cfg := startconditions.DefaultConfig()
	cfg.Waypoints.NarrowCenterBiasM = 0 // isolate the geometry from the bias split
	cfg.Waypoints.WideCenterBiasM = 0
	const widthM = 1.0
	widths := uniformWidths(widthM)
	trackCenter := cfg.TrackMaxCoordM / 2

	farEdge := cfg.TrackMaxCoordM - widthM/2

	cases := []struct {
		name         string
		section      trackmodel.Section
		direction    trackmodel.Direction
		wantX, wantY float64
		wantYaw      float64
	}{
		{"south_ccw", trackmodel.South, trackmodel.Counterclockwise, trackCenter, widthM / 2, 0},
		{"south_cw", trackmodel.South, trackmodel.Clockwise, trackCenter, widthM / 2, math.Pi},
		{"north_ccw", trackmodel.North, trackmodel.Counterclockwise, trackCenter, farEdge, math.Pi},
		{"north_cw", trackmodel.North, trackmodel.Clockwise, trackCenter, farEdge, 0},
		{"east_cw", trackmodel.East, trackmodel.Clockwise, farEdge, trackCenter, -math.Pi / 2},
		{
			"east_ccw",
			trackmodel.East,
			trackmodel.Counterclockwise,
			farEdge,
			trackCenter,
			math.Pi / 2,
		},
		{
			"west_ccw",
			trackmodel.West,
			trackmodel.Counterclockwise,
			widthM / 2,
			trackCenter,
			-math.Pi / 2,
		},
		{"west_cw", trackmodel.West, trackmodel.Clockwise, widthM / 2, trackCenter, math.Pi / 2},
	}

	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()

			x, y, yaw, ok := startconditions.StartPose(tc.section, tc.direction, widths, cfg, nil)

			if !ok {
				t.Fatalf("StartPose(%v, %v) = ok false, want true", tc.section, tc.direction)
			}
			if math.Abs(x-tc.wantX) > 1e-9 || math.Abs(y-tc.wantY) > 1e-9 {
				t.Errorf("pose = (%.6f, %.6f), want (%.6f, %.6f)", x, y, tc.wantX, tc.wantY)
			}
			normalized := math.Mod(yaw-tc.wantYaw+math.Pi, 2*math.Pi) - math.Pi
			if math.Abs(normalized) > 1e-9 {
				t.Errorf("yaw = %v, want %v", yaw, tc.wantYaw)
			}
		})
	}
}

// TestStartPose_CenterBiasShiftsTowardInner checks the bias moves the
// centerline toward the inner block, matching CenterBiasForCorridor's
// documented sign convention (positive = toward inner).
func TestStartPose_CenterBiasShiftsTowardInner(t *testing.T) {
	t.Parallel()

	cfg := startconditions.DefaultConfig()
	widths := uniformWidths(cfg.NarrowWidthM) // narrow, so NarrowCenterBiasM applies

	_, yBiased, _, ok := startconditions.StartPose(
		trackmodel.South,
		trackmodel.Counterclockwise,
		widths,
		cfg,
		nil,
	)
	if !ok {
		t.Fatal("StartPose = ok false")
	}

	unbiased := cfg.NarrowWidthM / 2
	bias := cfg.Waypoints.NarrowCenterBiasM
	// South's inner block is toward +y, so a positive bias increases y.
	wantBiased := unbiased + bias
	if math.Abs(yBiased-wantBiased) > 1e-9 {
		t.Errorf("y = %v, want %v (unbiased %v + bias %v)", yBiased, wantBiased, unbiased, bias)
	}
}

// TestStartPose_CenterBiasOverride checks a non-nil centerBiasM pins every
// corridor to that magnitude, skipping the narrow/wide split -- the
// Obstacles Challenge's usage.
func TestStartPose_CenterBiasOverride(t *testing.T) {
	t.Parallel()

	cfg := startconditions.DefaultConfig()
	widths := uniformWidths(cfg.NarrowWidthM)
	override := 0.05

	_, y, _, ok := startconditions.StartPose(
		trackmodel.South,
		trackmodel.Counterclockwise,
		widths,
		cfg,
		&override,
	)
	if !ok {
		t.Fatal("StartPose = ok false")
	}

	want := cfg.NarrowWidthM/2 + override
	if math.Abs(y-want) > 1e-9 {
		t.Errorf("y = %v, want %v", y, want)
	}
}

// TestStartPose_UnknownSectionRefuses checks an out-of-range section is
// refused rather than silently returning a zero pose.
func TestStartPose_UnknownSectionRefuses(t *testing.T) {
	t.Parallel()

	cfg := startconditions.DefaultConfig()
	widths := uniformWidths(1.0)

	x, y, yaw, ok := startconditions.StartPose(
		trackmodel.Section(99),
		trackmodel.Clockwise,
		widths,
		cfg,
		nil,
	)

	if ok {
		t.Errorf(
			"StartPose with an unknown section = ok true (%v, %v, %v), want ok false",
			x,
			y,
			yaw,
		)
	}
}

// TestAssumedStartConditions_DefaultsToNarrowWidths checks a nil widths map
// falls back to cfg.NarrowWidthM on every side -- the safe prior blind
// operation starts from.
func TestAssumedStartConditions_DefaultsToNarrowWidths(t *testing.T) {
	t.Parallel()

	cfg := startconditions.DefaultConfig()

	withNil, ok := startconditions.AssumedStartConditions(
		trackmodel.Counterclockwise, nil, startconditions.CanonicalSection, cfg, nil,
	)
	if !ok {
		t.Fatal("AssumedStartConditions(nil widths) = ok false")
	}
	withExplicit, ok := startconditions.AssumedStartConditions(
		trackmodel.Counterclockwise,
		uniformWidths(cfg.NarrowWidthM),
		startconditions.CanonicalSection,
		cfg,
		nil,
	)
	if !ok {
		t.Fatal("AssumedStartConditions(explicit narrow widths) = ok false")
	}

	if withNil != withExplicit {
		t.Errorf(
			"nil-widths result %+v != explicit-narrow-widths result %+v",
			withNil,
			withExplicit,
		)
	}
	if withNil.Section != startconditions.CanonicalSection {
		t.Errorf("Section = %v, want CanonicalSection", withNil.Section)
	}
	if withNil.Direction != trackmodel.Counterclockwise {
		t.Errorf("Direction = %v, want Counterclockwise", withNil.Direction)
	}
}

// TestCanonicalSection_IsSouth checks the package constant matches
// shared.domain.enums.Section.canonical()'s hardcoded SOUTH.
func TestCanonicalSection_IsSouth(t *testing.T) {
	t.Parallel()

	if startconditions.CanonicalSection != trackmodel.South {
		t.Errorf("CanonicalSection = %v, want trackmodel.South", startconditions.CanonicalSection)
	}
}
