package controllers_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// There is no dedicated Python oracle test file for NavigationTuning's
// from_tuning wiring itself -- every Python test file exercises it only
// indirectly, via a `controller = CollisionAvoidanceController.from_tuning(
// NavigationTuning.load_default())` (or WaypointController's/StuckDetector's
// equivalent) fixture, which this package's newDefaultCollisionAvoidance
// Controller/newDefaultWaypointController helpers already mirror throughout
// the other test files. These pin the handful of documented, deterministic
// conversions config.go's own doc comments make explicit, which those
// fixtures alone don't exercise directly (a wrong pass-through value would
// still make some *other* test fail, but not point at the actual bug).

// TestNewCollisionAvoidanceController_PassesThroughClearanceThresholds pins
// that Config's clearance fields land unchanged on the controller, matching
// CollisionAvoidanceController.from_tuning's direct field assignments.
func TestNewCollisionAvoidanceController_PassesThroughClearanceThresholds(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	controller := cfg.NewCollisionAvoidanceController()

	if controller.ContactDist != cfg.ContactDist {
		t.Errorf("ContactDist = %v, want %v", controller.ContactDist, cfg.ContactDist)
	}
	if controller.SlowDist != cfg.SlowDist {
		t.Errorf("SlowDist = %v, want %v", controller.SlowDist, cfg.SlowDist)
	}
	if controller.FastDist != cfg.FastDist {
		t.Errorf("FastDist = %v, want %v", controller.FastDist, cfg.FastDist)
	}
}

// TestNewCollisionAvoidanceController_PathHalfWidthIsChassisHalfWidthPlusMargin
// pins the one derived (non-pass-through) field among the clearance group:
// PathHalfWidth = ChassisWidthM/2 + PathMargin, matching from_tuning's own
// computation of assess_risk's forward-lane half-width.
func TestNewCollisionAvoidanceController_PathHalfWidthIsChassisHalfWidthPlusMargin(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	controller := cfg.NewCollisionAvoidanceController()

	want := cfg.ChassisWidthM/2.0 + cfg.PathMargin
	if math.Abs(controller.PathHalfWidth-want) > 1e-12 {
		t.Errorf("PathHalfWidth = %v, want %v", controller.PathHalfWidth, want)
	}
}

// TestNewCollisionAvoidanceController_EscapeSteerScaleClampsAtFullLock pins
// angleRadToNorm's clamp: a rev_steer_deg tuned larger than the physical
// steering limit must saturate at 1.0 rather than overshoot it, matching
// shared.domain.steering.angle_rad_to_steering_norm's own clamp (see
// config.go's angleRadToNorm doc comment).
func TestNewCollisionAvoidanceController_EscapeSteerScaleClampsAtFullLock(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	cfg.MaxSteeringAngleRad = 0.1 // far below RevSteerDeg's 44deg (~0.768 rad)
	controller := cfg.NewCollisionAvoidanceController()

	if controller.EscapeSteerScale != 1.0 {
		t.Errorf("EscapeSteerScale = %v, want 1.0 (clamped at full lock)", controller.EscapeSteerScale)
	}
}

// TestNewCollisionAvoidanceController_ZeroMaxSteeringAngleYieldsZeroScale
// pins angleRadToNorm's other documented edge: a non-positive
// MaxSteeringAngleRad (an unconfigured hardware profile) must not divide by
// zero or produce Inf/NaN, returning 0.0 instead.
func TestNewCollisionAvoidanceController_ZeroMaxSteeringAngleYieldsZeroScale(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	cfg.MaxSteeringAngleRad = 0.0
	controller := cfg.NewCollisionAvoidanceController()

	if controller.EscapeSteerScale != 0.0 {
		t.Errorf("EscapeSteerScale = %v, want 0.0", controller.EscapeSteerScale)
	}
	if controller.SideCorrectionSteer != 0.0 {
		t.Errorf("SideCorrectionSteer = %v, want 0.0", controller.SideCorrectionSteer)
	}
}

// TestNewStuckDetector_HistorySizeIsDoubleTimeoutOrFloor pins
// Config.NewStuckDetector's documented historySize rule (matching
// StuckDetector.from_tuning): max(timeout_frames*2, stuck_history_floor).
func TestNewStuckDetector_HistorySizeIsDoubleTimeoutOrFloor(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	cfg.StuckTimeoutFrames = 5 // 5*2=10, well under DefaultStuckHistoryFloor (60)
	detector, err := cfg.NewStuckDetector(nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}
	if detector.HistorySize != cfg.StuckHistoryFloor {
		t.Errorf("HistorySize = %v, want the floor %v", detector.HistorySize, cfg.StuckHistoryFloor)
	}

	cfg.StuckTimeoutFrames = 40 // 40*2=80, over the floor
	detector, err = cfg.NewStuckDetector(nil)
	if err != nil {
		t.Fatalf("NewStuckDetector() err = %v", err)
	}
	if detector.HistorySize != 80 {
		t.Errorf("HistorySize = %v, want 80 (2x timeout)", detector.HistorySize)
	}
}

// TestDefaultConfig_MatchesShippedTOMLDefaults spot-checks a handful of
// DefaultConfig's literals against the shipped TOML values its own doc
// comment claims to mirror (platform/shared/config/navigation/**), so a
// future edit to one without the other doesn't drift silently.
func TestDefaultConfig_MatchesShippedTOMLDefaults(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()

	cases := map[string]struct{ got, want float64 }{
		"ContactDist":            {cfg.ContactDist, 0.10},
		"SlowDist":               {cfg.SlowDist, 0.25},
		"LookaheadBlendStart":    {cfg.LookaheadBlendStart, 0.70},
		"CornerTurnThresholdRad": {cfg.CornerTurnThresholdRad, 0.35},
		"WheelbaseM":             {cfg.WheelbaseM, 0.19},
		"ChassisWidthM":          {cfg.ChassisWidthM, 0.194},
	}
	for name, c := range cases {
		if c.got != c.want {
			t.Errorf("%s = %v, want %v", name, c.got, c.want)
		}
	}
}
