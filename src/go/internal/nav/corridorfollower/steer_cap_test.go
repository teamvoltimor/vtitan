package corridorfollower_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/corridorfollower"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Ports corridor_follower.steer_cap_norm, which has no Python test of its
// own -- these pin the behaviour the Python docstring argues for.

const steerCapTolerance = 1e-9

// At the anchor distance the cap must be MaxCornerSteerDeg exactly, so the
// measured wide-corner case is untouched by the re-derivation.
func TestSteerCapNormIsTheAnchorAtTheAnchorDistance(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()

	got := corridorfollower.SteerCapNorm(cfg.TurnClearanceM, cfg)
	want := navutil.SteeringNormFromAngleRad(
		cfg.MaxCornerSteerDeg*math.Pi/navutil.DegreesPerHalfTurn, cfg.MaxSteeringAngleRad,
	)
	if math.Abs(got-want) > steerCapTolerance {
		t.Errorf("at the anchor distance: got %v, want %v", got, want)
	}
}

// A branch that commits closer needs a tighter radius, which is a LARGER
// steering angle. Sharing one constant across branches is the defect this
// exists to fix: the back-off branch commits at 0.30 m, half the corner
// branch's 0.60 m, and drove the corner's arc there.
func TestSteerCapNormTightensAsTheCommitDistanceShortens(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()

	corner := corridorfollower.SteerCapNorm(cfg.TurnClearanceM, cfg)
	narrow := corridorfollower.SteerCapNorm(cfg.NarrowTurnClearanceM, cfg)
	backoff := corridorfollower.SteerCapNorm(cfg.MinForwardClearanceM, cfg)

	if !(corner < narrow && narrow < backoff) {
		t.Errorf(
			"caps must grow as the commit distance shortens: corner=%v narrow=%v backoff=%v",
			corner, narrow, backoff,
		)
	}
}

// tan(cap) = tan(anchor) * TurnClearanceM / d -- the ratio form the docstring
// states, checked against a distance neither branch uses.
func TestSteerCapNormFollowsTheRatioForm(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()
	const commitM = 0.45

	anchorRad := cfg.MaxCornerSteerDeg * math.Pi / navutil.DegreesPerHalfTurn
	wantRad := math.Atan(math.Tan(anchorRad) * cfg.TurnClearanceM / commitM)
	want := navutil.SteeringNormFromAngleRad(wantRad, cfg.MaxSteeringAngleRad)

	if got := corridorfollower.SteerCapNorm(commitM, cfg); math.Abs(got-want) > steerCapTolerance {
		t.Errorf("got %v, want %v", got, want)
	}
}

// Disabling the flag must restore the single shared anchor for every branch,
// so the pre-fix behaviour stays reachable for an A/B.
func TestSteerCapNormDisabledReturnsTheAnchorEverywhere(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()
	cfg.SteerCapFromCommitDistance = false

	want := navutil.SteeringNormFromAngleRad(
		cfg.MaxCornerSteerDeg*math.Pi/navutil.DegreesPerHalfTurn, cfg.MaxSteeringAngleRad,
	)
	for _, commitM := range []float64{0.30, 0.45, 0.60, 1.20} {
		if got := corridorfollower.SteerCapNorm(commitM, cfg); math.Abs(got-want) > steerCapTolerance {
			t.Errorf("disabled, commit %v: got %v, want %v", commitM, got, want)
		}
	}
}

// A non-positive commit distance would divide by zero. Fall back to the
// anchor rather than producing an infinite curvature.
func TestSteerCapNormNonPositiveCommitFallsBackToTheAnchor(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()

	want := navutil.SteeringNormFromAngleRad(
		cfg.MaxCornerSteerDeg*math.Pi/navutil.DegreesPerHalfTurn, cfg.MaxSteeringAngleRad,
	)
	for _, commitM := range []float64{0.0, -0.5} {
		if got := corridorfollower.SteerCapNorm(commitM, cfg); math.Abs(got-want) > steerCapTolerance {
			t.Errorf("commit %v: got %v, want %v", commitM, got, want)
		}
	}
}

// The cap is a steering magnitude, so it can never exceed full lock however
// short the commit distance gets.
func TestSteerCapNormNeverExceedsFullLock(t *testing.T) {
	cfg := corridorfollower.DefaultConfig()

	for _, commitM := range []float64{0.30, 0.10, 0.01, 0.001} {
		if got := corridorfollower.SteerCapNorm(commitM, cfg); got > 1.0+steerCapTolerance {
			t.Errorf("commit %v: got %v, want <= 1.0", commitM, got)
		}
	}
}
