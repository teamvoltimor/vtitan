package harness

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

// ApplyRobotProfile overlays the shipped robot.toml sensor and chassis spec
// onto cfg, returning the result. An empty configRoot returns cfg unchanged,
// matching every other ConfigFor in the tree ("no root" means "run on the
// literal defaults").
//
// These fields were hardcoded in DefaultConfig with values that did not match
// the robot the rest of the stack is configured for, and --config-root did not
// reach them. The LIDAR floor shipped above robot.toml's own minimum, so the
// simulated sensor went blind further out than the real C1 does, exactly
// across the range where the chassis works closest to a sign, and the
// Obstacles corpus collided far more than the Python oracle with contact
// concentrated on obstacles within the first lap. See
// adr:0068-go-parallel-track-single-cutover.
//
// Deliberately NOT sourced here: LidarNoiseStd and InvalidRayRate. robot.toml
// carries a noise_stddev, but the noise/dropout pair are simulation parameters
// whose current values are a baseline; moving them is a re-baselining decision
// rather than a parity fix. See adr:0087-test-methodology.
func ApplyRobotProfile(
	logger *slog.Logger, cfg Config, configRoot string, hardwareProfileNames []string,
) Config {
	if configRoot == "" {
		return cfg
	}

	robotPath := filepath.Join(configRoot, profile.DefaultRobotTOMLPath)
	loaded, err := profile.LoadRobotConfig(robotPath, hardwareProfileNames)
	if err != nil {
		logger.Warn("harness: loading robot.toml, keeping default sensor/chassis spec",
			"config_root", configRoot, "error", err)
		return cfg
	}

	// Each guarded on being positive: a section absent from the TOML
	// deserializes to a zero that would otherwise install a 0 m LIDAR or a
	// zero-width chassis, which is strictly worse than the default it replaced.
	if loaded.Lidar.MountXOffset > 0 {
		cfg.LidarMountXOffsetM = loaded.Lidar.MountXOffset
	}
	if loaded.Lidar.MinRange > 0 {
		cfg.LidarMinRangeM = loaded.Lidar.MinRange
	}
	if loaded.Lidar.MaxRange > 0 {
		cfg.LidarMaxRangeM = loaded.Lidar.MaxRange
	}
	if loaded.Lidar.Samples > 0 {
		cfg.LidarSamples = loaded.Lidar.Samples
	}
	if loaded.Lidar.UpdateRate > 0 {
		cfg.LidarHz = loaded.Lidar.UpdateRate
	}
	if loaded.Chassis.Length > 0 {
		cfg.ChassisLengthM = loaded.Chassis.Length
	}
	if loaded.Chassis.Width > 0 {
		cfg.ChassisWidthM = loaded.Chassis.Width
	}
	return cfg
}
