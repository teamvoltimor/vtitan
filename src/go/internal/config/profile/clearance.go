package profile

// ClearanceConfig mirrors
// src/config/navigation/motion/clearance.toml
// (shared.config.navigation_tuning.motion.ClearanceZones) in full.
// internal/nav/controllers.Config consumes ContactDist/SlowDist/FastDist/
// PathMargin; MediumDist is mirrored for completeness even though nothing
// in internal/nav/controllers reads it yet (CollisionAvoidanceController's
// speed-zone ladder lives in the not-yet-ported core navigator).
type ClearanceConfig struct {
	// ContactDist matches CONTACT_DIST -- robot creeps forward below this (m).
	ContactDist float64 `mapstructure:"contact_dist"`
	// ObstaclesContactDist matches OBSTACLES_CONTACT_DIST -- the
	// Obstacles-Challenge-only contact zone, which supersedes ContactDist
	// whenever a sign router is attached (see
	// controllers.Config.ForObstaclesChallenge). A pointer because the key is
	// genuinely optional, exactly as it is optional in Python: absent must
	// mean "leave ContactDist alone", and a zero float64 would instead mean
	// "escape never fires", silently disabling the gate on any config that
	// omits the key.
	ObstaclesContactDist *float64 `mapstructure:"obstacles_contact_dist"`
	// SlowDist matches SLOW_DIST -- reduced-speed zone (m).
	SlowDist float64 `mapstructure:"slow_dist"`
	// MediumDist matches MEDIUM_DIST -- normal-speed zone (m).
	MediumDist float64 `mapstructure:"medium_dist"`
	// FastDist matches FAST_DIST -- full-speed capability beyond this (m).
	FastDist float64 `mapstructure:"fast_dist"`
	// PathMargin matches PATH_MARGIN -- extra clearance beyond the chassis
	// half-width still counted as "in the robot's forward path" (m).
	PathMargin float64 `mapstructure:"path_margin"`
	// RiskRayWindow matches RISK_RAY_WINDOW: how many ADJACENT lane rays must
	// corroborate a short return before AssessRisk treats it as an obstacle.
	// 1 is the bare minimum, which a noisy sweep turns into a phantom.
	RiskRayWindow int `mapstructure:"risk_ray_window"`
	// ForwardNoDataIsDegraded matches FORWARD_NO_DATA_IS_DEGRADED: when every
	// ray in the forward sector is invalid (a wall too close to return a
	// signal reads as NO_DATA_RANGE_M, indistinguishable from open road),
	// treat that exactly like having no LIDAR at all rather than as measured
	// clearance. Ships true; the `default` tag keeps a TOML that omits the
	// key from silently disabling the gate via the zero value.
	ForwardNoDataIsDegraded bool `mapstructure:"forward_no_data_is_degraded" default:"true"`
	// ForwardPathAheadOfBumper matches FORWARD_PATH_AHEAD_OF_BUMPER: measure
	// the forward driving lane from the front bumper face rather than from
	// the LIDAR. Ships false (the LIDAR-relative test, which
	// ForwardPathRanges already implements); true is an untested alternate
	// this port does not implement -- see
	// controllers.Config.ForwardPathAheadOfBumper's doc comment.
	ForwardPathAheadOfBumper bool `mapstructure:"forward_path_ahead_of_bumper"`
}

// DefaultClearanceTOMLPath is
// src/config/navigation/motion/clearance.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultClearanceTOMLPath = "src/config/navigation/motion/clearance.toml"
