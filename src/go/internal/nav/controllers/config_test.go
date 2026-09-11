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
// navutil.SteeringNormFromAngleRad's clamp: a rev_steer_deg tuned larger than the physical
// steering limit must saturate at 1.0 rather than overshoot it, matching
// shared.domain.steering.angle_rad_to_steering_norm's own clamp.
func TestNewCollisionAvoidanceController_EscapeSteerScaleClampsAtFullLock(t *testing.T) {
	t.Parallel()

	cfg := controllers.DefaultConfig()
	cfg.MaxSteeringAngleRad = 0.1 // far below RevSteerDeg's 44deg (~0.768 rad)
	controller := cfg.NewCollisionAvoidanceController()

	if controller.EscapeSteerScale != 1.0 {
		t.Errorf(
			"EscapeSteerScale = %v, want 1.0 (clamped at full lock)",
			controller.EscapeSteerScale,
		)
	}
}

// TestNewCollisionAvoidanceController_ZeroMaxSteeringAngleYieldsZeroScale
// pins navutil.SteeringNormFromAngleRad's other documented edge: a non-positive
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

// TestForObstaclesChallenge mirrors the Python oracle
// tests/unit/test_navigation_tuning.py's coverage of
// ClearanceZones.for_obstacles_challenge: the override supersedes the shared
// contact zone when set, and is the exact identity when it is not.
//
// The identity half is the one worth pinning. It is what keeps the Open
// Challenge byte-identical to the un-ported path, and a regression there
// would not show up as a wrong number -- it would show up as Open quietly
// escaping on a threshold nothing measured it against.
func TestForObstaclesChallenge(t *testing.T) {
	t.Parallel()

	base := controllers.DefaultConfig()
	if got := base.ForObstaclesChallenge(); got != base {
		t.Errorf("ForObstaclesChallenge() with no override = %+v, want the receiver unchanged", got)
	}

	const override = 0.05
	withOverride := base
	withOverride.ObstaclesContactDist = &[]float64{override}[0]
	got := withOverride.ForObstaclesChallenge()
	if got.ContactDist != override {
		t.Errorf("ForObstaclesChallenge().ContactDist = %v, want %v", got.ContactDist, override)
	}
	if withOverride.ContactDist != base.ContactDist {
		t.Errorf("ForObstaclesChallenge() mutated its receiver: ContactDist = %v, want %v",
			withOverride.ContactDist, base.ContactDist)
	}
	// The escape gate is the only zone the override moves; a wider blast
	// radius would change speed-ladder rungs the 256-corpus A/B never varied.
	if got.SlowDist != base.SlowDist || got.FastDist != base.FastDist {
		t.Errorf("ForObstaclesChallenge() moved a non-contact zone: SlowDist/FastDist = %v/%v, want %v/%v",
			got.SlowDist, got.FastDist, base.SlowDist, base.FastDist)
	}
}

// The Open Challenge's straight lookahead REPLACES the base one, so an Open
// run does not silently drive the Obstacles value.
func TestForOpenChallenge_ReplacesTheStraightLookahead(t *testing.T) {
	cfg := controllers.DefaultConfig()

	open := cfg.ForOpenChallenge()

	if open.LookaheadLong != cfg.OpenLookaheadLong {
		t.Errorf("LookaheadLong = %v, want the Open override %v",
			open.LookaheadLong, cfg.OpenLookaheadLong)
	}
	if open.LookaheadShort != cfg.LookaheadShort {
		t.Errorf("LookaheadShort = %v, want it untouched at %v",
			open.LookaheadShort, cfg.LookaheadShort)
	}
}

// Obstacles reads the base parameters: the override must not be able to
// shadow the base constant on an Obstacles sweep.
func TestForOpenChallenge_LeavesTheBaseConfigUnchanged(t *testing.T) {
	cfg := controllers.DefaultConfig()
	before := cfg.LookaheadLong

	_ = cfg.ForOpenChallenge()

	if cfg.LookaheadLong != before {
		t.Errorf("base LookaheadLong = %v, want %v", cfg.LookaheadLong, before)
	}
}

// With no override configured the resolution is the identity, so a caller
// need not branch on whether one is set.
func TestForOpenChallenge_IsTheIdentityWithoutAnOverride(t *testing.T) {
	cfg := controllers.DefaultConfig()
	cfg.OpenLookaheadLong = 0.0

	if got := cfg.ForOpenChallenge(); got.LookaheadLong != cfg.LookaheadLong {
		t.Errorf("LookaheadLong = %v, want the base %v", got.LookaheadLong, cfg.LookaheadLong)
	}
}
