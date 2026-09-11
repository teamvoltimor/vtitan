package encoder_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/encoder"
)

// shippedCountsPerRev is the live-verified 2026-08-29 calibration for the
// current motor (60, not the earlier 86 -- the older value made the drive
// ceiling read as ~0.45 m/s when it is ~0.58).
const shippedCountsPerRev = 60.0

// shippedWheelDiameterM is robot.toml's wheel radius (0.035) doubled, the
// same derivation Python's calibration.DEFAULT_WHEEL_DIAMETER_M makes.
const shippedWheelDiameterM = 0.07

func TestCountsToRevolutions(t *testing.T) {
	got, err := encoder.CountsToRevolutions(120, shippedCountsPerRev)
	if err != nil {
		t.Fatalf("CountsToRevolutions: %v", err)
	}
	if math.Abs(got-2.0) > 1e-12 {
		t.Fatalf("revolutions = %g, want 2", got)
	}
}

func TestCountsToRevolutions_RejectsNonPositiveCountsPerRev(t *testing.T) {
	// A missing motor profile leaves counts_per_rev at zero. It must error
	// rather than divide, since a NaN/Inf distance propagates silently
	// through the whole odometry chain.
	if _, err := encoder.CountsToRevolutions(120, 0); err == nil {
		t.Fatal("CountsToRevolutions(counts_per_rev=0): want error, got nil")
	}
}

func TestCountsToDistance_MatchesCircumference(t *testing.T) {
	got, err := encoder.CountsToDistance(
		shippedCountsPerRev, shippedCountsPerRev, shippedWheelDiameterM,
	)
	if err != nil {
		t.Fatalf("CountsToDistance: %v", err)
	}
	want := math.Pi * shippedWheelDiameterM
	if math.Abs(got-want) > 1e-12 {
		t.Fatalf("one revolution = %g m, want %g", got, want)
	}
}

func TestCountsToDistance_SignedForReverse(t *testing.T) {
	got, err := encoder.CountsToDistance(
		-shippedCountsPerRev, shippedCountsPerRev, shippedWheelDiameterM,
	)
	if err != nil {
		t.Fatalf("CountsToDistance: %v", err)
	}
	// bayexit differences this value to decide the chassis has cleared its
	// pocket, and it commands REVERSE to get there -- an unsigned distance
	// would report the reverse leg as forward progress.
	if got >= 0 {
		t.Fatalf("reverse travel = %g m, want negative", got)
	}
}

func TestSpeedEstimator_HoldsUntilWindowFills(t *testing.T) {
	est, err := encoder.NewSpeedEstimator(shippedCountsPerRev)
	if err != nil {
		t.Fatalf("NewSpeedEstimator: %v", err)
	}

	// Seeding call, then one 20ms tick: well short of the 0.1s window.
	est.Update(0, 0.02)
	if got := est.Update(2, 0.02); got != 0 {
		t.Fatalf("rpm after one sub-window tick = %g, want 0 (held)", got)
	}
}

func TestSpeedEstimator_EmitsAfterWindow(t *testing.T) {
	est, err := encoder.NewSpeedEstimator(shippedCountsPerRev)
	if err != nil {
		t.Fatalf("NewSpeedEstimator: %v", err)
	}

	// 60 counts (one revolution) accumulated over 5 x 20ms = 0.1s is
	// 10 rev/s = 600 rpm raw; the first smoothed output is 0.3 of that.
	est.Update(0, 0.02)
	var rpm float64
	for tick := 1; tick <= 5; tick++ {
		rpm = est.Update(int64(tick*12), 0.02)
	}

	want := encoder.DefaultSmoothing * 600.0
	if math.Abs(rpm-want) > 1e-9 {
		t.Fatalf("rpm after full window = %g, want %g", rpm, want)
	}
	if got := est.RPM(); got != rpm {
		t.Fatalf("RPM() = %g, want the value Update returned (%g)", got, rpm)
	}
}

func TestSpeedEstimator_ReverseIsNegative(t *testing.T) {
	est, err := encoder.NewSpeedEstimator(shippedCountsPerRev)
	if err != nil {
		t.Fatalf("NewSpeedEstimator: %v", err)
	}

	est.Update(0, 0.02)
	var rpm float64
	for tick := 1; tick <= 5; tick++ {
		rpm = est.Update(int64(-tick*12), 0.02)
	}
	if rpm >= 0 {
		t.Fatalf("rpm reversing = %g, want negative", rpm)
	}
}

func TestSpeedEstimator_NonPositiveDTHoldsValue(t *testing.T) {
	est, err := encoder.NewSpeedEstimator(shippedCountsPerRev)
	if err != nil {
		t.Fatalf("NewSpeedEstimator: %v", err)
	}

	est.Update(0, 0.02)
	for tick := 1; tick <= 5; tick++ {
		est.Update(int64(tick*12), 0.02)
	}
	held := est.RPM()

	// Two samples in the same instant must not divide by zero, and must not
	// discard the window that was already accumulating.
	if got := est.Update(999, 0); got != held {
		t.Fatalf("rpm with dt=0 = %g, want the held %g", got, held)
	}
}

func TestSpeedEstimator_ResetClearsHistory(t *testing.T) {
	est, err := encoder.NewSpeedEstimator(shippedCountsPerRev)
	if err != nil {
		t.Fatalf("NewSpeedEstimator: %v", err)
	}

	est.Update(0, 0.02)
	for tick := 1; tick <= 5; tick++ {
		est.Update(int64(tick*12), 0.02)
	}
	est.Reset()

	if got := est.RPM(); got != 0 {
		t.Fatalf("RPM after Reset = %g, want 0", got)
	}
	// The first post-Reset Update re-seeds rather than differencing the
	// new count against the pre-reset one -- otherwise zeroing the hardware
	// counter would register as a huge backwards spike.
	if got := est.Update(0, 0.02); got != 0 {
		t.Fatalf("first Update after Reset = %g, want 0", got)
	}
}

func TestNewSpeedEstimatorWith_RejectsBadParameters(t *testing.T) {
	for _, tc := range []struct {
		name         string
		countsPerRev float64
		smoothing    float64
		minWindowS   float64
	}{
		{"zero counts per rev", 0, 0.3, 0.1},
		{"zero smoothing", 60, 0, 0.1},
		{"smoothing above one", 60, 1.5, 0.1},
		{"negative window", 60, 0.3, -0.1},
	} {
		t.Run(tc.name, func(t *testing.T) {
			_, err := encoder.NewSpeedEstimatorWith(encoder.SpeedEstimatorParams{
				CountsPerRev: tc.countsPerRev,
				Smoothing:    tc.smoothing,
				MinWindowS:   tc.minWindowS,
			})
			if err == nil {
				t.Fatal("want error, got nil")
			}
		})
	}
}

func TestConfig_ValidateRejectsIncompleteWiring(t *testing.T) {
	valid := encoder.Config{
		GPIOChip:       encoder.DefaultGPIOChip,
		PinA:           16,
		PinB:           20,
		CountsPerRev:   shippedCountsPerRev,
		WheelDiameterM: shippedWheelDiameterM,
	}
	if err := valid.Validate(); err != nil {
		t.Fatalf("valid config: %v", err)
	}

	for _, tc := range []struct {
		name   string
		mutate func(*encoder.Config)
	}{
		{"no chip", func(c *encoder.Config) { c.GPIOChip = "" }},
		{"same pin twice", func(c *encoder.Config) { c.PinB = c.PinA }},
		{"negative pin", func(c *encoder.Config) { c.PinA = -1 }},
		{"no calibration", func(c *encoder.Config) { c.CountsPerRev = 0 }},
		{"no wheel diameter", func(c *encoder.Config) { c.WheelDiameterM = 0 }},
	} {
		t.Run(tc.name, func(t *testing.T) {
			cfg := valid
			tc.mutate(&cfg)
			if err := cfg.Validate(); err == nil {
				t.Fatal("want error, got nil")
			}
		})
	}
}
