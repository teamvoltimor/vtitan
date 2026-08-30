package signrouter_test

import (
	"math"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/signrouter"
)

const configTolerance = 1e-9

// TestDefaultConfig_LateralOffsetIsHalfDiagonalPlusSignHalfWidthPlusMargin
// matches TestLateralOffsetTracksChassis
// .test_offset_is_half_diagonal_plus_sign_half_width_plus_margin: the
// shipped offset must be DERIVED from chassis + sign geometry, not a stale
// literal -- a 0.28mm chassis-width change was enough to flip a corpus
// scenario (see Config.LateralOffsetM's doc comment).
func TestDefaultConfig_LateralOffsetIsHalfDiagonalPlusSignHalfWidthPlusMargin(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	expected := math.Hypot(signrouter.DefaultChassisLengthM/2, signrouter.DefaultChassisWidthM/2) +
		signrouter.DefaultSignWidthM/2 + signrouter.DefaultSignClearanceMarginM

	if math.Abs(cfg.LateralOffsetM-expected) > configTolerance {
		t.Errorf("LateralOffsetM = %v, want %v", cfg.LateralOffsetM, expected)
	}
}

// TestDefaultConfig_OffsetUsesTheDiagonalNotTheWidth matches
// TestLateralOffsetTracksChassis.test_offset_uses_the_diagonal_not_the_width:
// a half-width-only derivation sizes a pass the robot can only make while
// already square, undersizing the offset for the ~2/3 of legal sign
// positions that sit on a corner boundary where the chassis is mid-turn.
func TestDefaultConfig_OffsetUsesTheDiagonalNotTheWidth(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	halfWidthDerivation := signrouter.DefaultChassisWidthM/2 + signrouter.DefaultSignWidthM/2 +
		signrouter.DefaultSignClearanceMarginM

	if halfWidthDerivation >= cfg.LateralOffsetM {
		t.Errorf(
			"half-width derivation %v must be < shipped LateralOffsetM %v",
			halfWidthDerivation,
			cfg.LateralOffsetM,
		)
	}
}

// TestDefaultConfig_WallClearanceTracksSameChassis matches
// TestLateralOffsetTracksChassis.test_wall_clearance_tracks_the_same_chassis.
func TestDefaultConfig_WallClearanceTracksSameChassis(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	expected := math.Hypot(signrouter.DefaultChassisLengthM/2, signrouter.DefaultChassisWidthM/2) +
		signrouter.DefaultWallClearanceMarginM

	got := cfg.ChassisHalfDiagonalM + cfg.WallClearanceMarginM
	if math.Abs(got-expected) > configTolerance {
		t.Errorf("chassis half-diagonal + wall clearance margin = %v, want %v", got, expected)
	}
}

// TestNewConfig_RejectsActivationAtOrAbovePassed matches
// TestActivationPassedOrdering.test_activation_at_or_above_passed_is_rejected:
// activation_dist must stay below passed_dist, or a sign is engaged and
// marked passed on the same tick -- measured on the 256-scenario corpus to
// silently take collisions from 209 to 256/256 with zero laps completed.
func TestNewConfig_RejectsActivationAtOrAbovePassed(t *testing.T) {
	t.Parallel()

	cases := []struct {
		name               string
		activation, passed float64
	}{
		{"the_measured_cliff", 1.30, 1.20},
		{"equal_is_just_as_broken", 1.20, 1.20},
		{"far_apart_but_still_inverted", 2.00, 0.50},
	}
	for _, tc := range cases {
		t.Run(tc.name, func(t *testing.T) {
			t.Parallel()
			cfg := signrouter.DefaultConfig()
			cfg.ActivationDistM = tc.activation
			cfg.PassedDistM = tc.passed

			_, err := signrouter.NewConfig(cfg)
			if err == nil {
				t.Fatalf(
					"NewConfig(activation=%v, passed=%v) = nil error, want an error",
					tc.activation,
					tc.passed,
				)
			}
			if !strings.Contains(err.Error(), "must be < passed_dist") {
				t.Errorf("error = %q, want it to mention %q", err.Error(), "must be < passed_dist")
			}
		})
	}
}

// TestNewConfig_AcceptsValidOrdering matches
// TestActivationPassedOrdering.test_valid_ordering_is_accepted.
func TestNewConfig_AcceptsValidOrdering(t *testing.T) {
	t.Parallel()

	cfg := signrouter.DefaultConfig()
	cfg.ActivationDistM = 1.60
	cfg.PassedDistM = 1.80

	got, err := signrouter.NewConfig(cfg)
	if err != nil {
		t.Fatalf("NewConfig() error = %v, want nil", err)
	}
	if !(got.ActivationDistM < got.PassedDistM) {
		t.Errorf(
			"ActivationDistM = %v, PassedDistM = %v, want activation < passed",
			got.ActivationDistM,
			got.PassedDistM,
		)
	}
}

// TestNewConfig_ShippedDefaultsSatisfyTheOrdering matches
// TestActivationPassedOrdering.test_shipped_defaults_satisfy_the_ordering.
func TestNewConfig_ShippedDefaultsSatisfyTheOrdering(t *testing.T) {
	t.Parallel()

	cfg, err := signrouter.NewConfig(signrouter.DefaultConfig())
	if err != nil {
		t.Fatalf("NewConfig(DefaultConfig()) error = %v, want nil", err)
	}
	if !(cfg.ActivationDistM < cfg.PassedDistM) {
		t.Errorf(
			"shipped defaults: ActivationDistM = %v, PassedDistM = %v, want activation < passed",
			cfg.ActivationDistM,
			cfg.PassedDistM,
		)
	}
}
