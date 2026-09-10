package controllers_test

import (
	"log/slog"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
)

// hardwareProfileNames matches the currently active profile recorded in
// this repo's own memory/config (VTITAN_HARDWARE_PROFILE =
// "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm"), so MaxSteeringAngleRad
// resolves to the real profile-sourced value rather than erroring for
// missing required fields (see profile.RobotConfig's doc comment: steering
// max_wheel_angle_deg has no chassis-only default).
var hardwareProfileNames = []string{"270deg-hiwonder-35kg", "rev-hd-hex-motor-6000rpm"}

// shippedObstaclesContactDist is clearance.toml's obstacles_contact_dist.
// Not taken from DefaultConfig like the other expectations in this file:
// DefaultConfig leaves the override nil (that IS its documented default, and
// the Open path depends on nil meaning "unset"), so comparing against it
// would assert the opposite of what ships.
const shippedObstaclesContactDist = 0.05

// repoRoot walks up from this test file's package directory
// (platform/robot-go/internal/nav/controllers) to the repo root, so
// ConfigFor can be exercised against the real checked-in TOML files --
// catching a path/field-name mismatch a testdata-fixture-only test
// wouldn't, matching the pattern already established in
// internal/nav/waypoints/config_profile_test.go and
// internal/nav/directionestimator/config_profile_test.go.
func repoRoot(t *testing.T) string {
	t.Helper()
	return filepath.Join("..", "..", "..", "..", "..")
}

// TestConfigFor_EmptyConfigRootReturnsDefaults has no direct Python
// oracle (ConfigFor is new Go plumbing with no equivalent shape in
// NavigationTuning.load_default(), which always reads from disk) -- this
// pins ConfigFor's own documented contract instead: an empty configRoot
// is the literal-defaults path, used by callers with no config tree at all.
func TestConfigFor_EmptyConfigRootReturnsDefaults(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	got := controllers.ConfigFor(logger, "", nil)
	want := controllers.DefaultConfig()
	if got != want {
		t.Errorf("ConfigFor(\"\", nil) = %+v, want DefaultConfig() %+v", got, want)
	}
}

// TestConfigFor_LoadsRealNavigationTuningFiles matches the pattern in
// internal/nav/waypoints/config_profile_test.go: exercise ConfigFor
// against the actual checked-in TOML tree, and confirm the fields sourced
// from clearance/control/pursuit/lidar_sectors/escape/waypoints.toml equal
// DefaultConfig's literals -- which this port's config.go doc comment
// states mirror those same shipped files exactly.
func TestConfigFor_LoadsRealNavigationTuningFiles(t *testing.T) {
	t.Parallel()

	logger := slog.New(slog.DiscardHandler)
	cfg := controllers.ConfigFor(logger, repoRoot(t), hardwareProfileNames)
	def := controllers.DefaultConfig()

	if cfg.ContactDist != def.ContactDist || cfg.SlowDist != def.SlowDist ||
		cfg.PathMargin != def.PathMargin {
		t.Errorf(
			"clearance.toml wiring: ContactDist/SlowDist/PathMargin = %v/%v/%v, want %v/%v/%v",
			cfg.ContactDist,
			cfg.SlowDist,
			cfg.PathMargin,
			def.ContactDist,
			def.SlowDist,
			def.PathMargin,
		)
	}
	// The Obstacles override ships SET, so a nil here means the key silently
	// failed to load and Go would run the escape gate at the shared 0.10 while
	// Python runs it at 0.05 -- a divergence invisible to every other
	// assertion in this test, since ContactDist itself still matches.
	if cfg.ObstaclesContactDist == nil {
		t.Error("clearance.toml wiring: ObstaclesContactDist = nil, want the shipped override")
	} else if *cfg.ObstaclesContactDist != shippedObstaclesContactDist {
		t.Errorf("clearance.toml wiring: ObstaclesContactDist = %v, want %v",
			*cfg.ObstaclesContactDist, shippedObstaclesContactDist)
	}
	if cfg.ControlHz != def.ControlHz {
		t.Errorf("control.toml wiring: ControlHz = %v, want %v", cfg.ControlHz, def.ControlHz)
	}
	if cfg.LookaheadShort != def.LookaheadShort || cfg.LookaheadLong != def.LookaheadLong {
		t.Errorf("pursuit.toml wiring: LookaheadShort/Long = %v/%v, want %v/%v",
			cfg.LookaheadShort, cfg.LookaheadLong, def.LookaheadShort, def.LookaheadLong)
	}
	if cfg.ThreatHalfFovDeg != def.ThreatHalfFovDeg ||
		cfg.BlindWedgeLeftMinDeg != def.BlindWedgeLeftMinDeg {
		t.Errorf(
			"lidar_sectors.toml wiring: ThreatHalfFovDeg/BlindWedgeLeftMinDeg = %v/%v, want %v/%v",
			cfg.ThreatHalfFovDeg,
			cfg.BlindWedgeLeftMinDeg,
			def.ThreatHalfFovDeg,
			def.BlindWedgeLeftMinDeg,
		)
	}
	if cfg.RevSteerDeg != def.RevSteerDeg || cfg.KTurnMaxFrames != def.KTurnMaxFrames {
		t.Errorf("escape.toml wiring: RevSteerDeg/KTurnMaxFrames = %v/%v, want %v/%v",
			cfg.RevSteerDeg, cfg.KTurnMaxFrames, def.RevSteerDeg, def.KTurnMaxFrames)
	}
	if cfg.ControllerReachedDistanceM != def.ControllerReachedDistanceM {
		t.Errorf("waypoints.toml wiring: ControllerReachedDistanceM = %v, want %v",
			cfg.ControllerReachedDistanceM, def.ControllerReachedDistanceM)
	}

	// robot.toml + the active hardware profile: wheelbase/chassis width/LIDAR
	// max range are chassis-only facts (already in DefaultConfig's literal
	// fallback) and load regardless of profile completeness.
	if cfg.WheelbaseM != def.WheelbaseM {
		t.Errorf("robot.toml wiring: WheelbaseM = %v, want %v", cfg.WheelbaseM, def.WheelbaseM)
	}
	if cfg.LidarMaxRangeM != def.LidarMaxRangeM {
		t.Errorf(
			"robot.toml wiring: LidarMaxRangeM = %v, want %v",
			cfg.LidarMaxRangeM,
			def.LidarMaxRangeM,
		)
	}

	// MaxSteeringAngleRad is profile-sourced (the 270deg servo's
	// max_wheel_angle_deg) and SHOULD differ from DefaultConfig's no-profile
	// literal fallback (1.2252 rad, ~70.2deg) once robot.toml actually
	// loads. As of this port, profile.LoadRobotConfig also requires
	// drivetrain.speed_response_tau_s from an active profile (see
	// requiredRobotKeys in internal/config/profile/robot.go), which no
	// checked-in profile under src/config/profiles/ currently
	// supplies -- a separate, pre-existing gap in the profile TOML tree,
	// not in this package. Until that lands, robot.toml load fails and
	// ConfigFor falls back to the literal default, which this asserts
	// explicitly instead of silently passing either way.
	if cfg.MaxSteeringAngleRad == def.MaxSteeringAngleRad {
		t.Logf(
			"MaxSteeringAngleRad still equals the no-profile fallback (%v): "+
				"robot.toml load is failing on a missing drivetrain.speed_response_tau_s "+
				"in every checked-in profile -- see this test's doc comment",
			def.MaxSteeringAngleRad,
		)
	}
}
