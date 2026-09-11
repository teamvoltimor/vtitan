package profile

// CorridorFollowerConfig mirrors
// src/config/navigation/blind_nav/corridor_follower.toml
// (shared.config.navigation_tuning.blind_nav.CorridorFollowerParams) in
// full.
//
// It once mirrored only the three fields
// internal/nav/directionestimator.DirectionFromParkingBay consumes, on the
// grounds that corridor_follower.py's own blind-creep/corner-turn control
// loop was not ported. It since was -- internal/nav/corridorfollower -- and
// the mirror was not extended with it, so every one of those constants read
// a Go literal and no edit to this TOML reached the running follower.
type CorridorFollowerConfig struct {
	// MinForwardClearanceM matches MIN_FORWARD_CLEARANCE_M.
	MinForwardClearanceM float64 `mapstructure:"min_forward_clearance_m"`
	// TurnClearanceM matches TURN_CLEARANCE_M.
	TurnClearanceM float64 `mapstructure:"turn_clearance_m"`
	// NarrowTurnClearanceM matches NARROW_TURN_CLEARANCE_M.
	NarrowTurnClearanceM float64 `mapstructure:"narrow_turn_clearance_m"`
	// CenteringGainDegPerM matches CENTERING_GAIN_DEG_PER_M.
	CenteringGainDegPerM float64 `mapstructure:"centering_gain_deg_per_m"`
	// HeadingGain matches HEADING_GAIN.
	HeadingGain float64 `mapstructure:"heading_gain"`
	// MaxCenteringSteerDeg matches MAX_CENTERING_STEER_DEG.
	MaxCenteringSteerDeg float64 `mapstructure:"max_centering_steer_deg"`
	// MaxCornerSteerDeg matches MAX_CORNER_STEER_DEG.
	MaxCornerSteerDeg float64 `mapstructure:"max_corner_steer_deg"`
	// SteerCapFromCommitDistance matches STEER_CAP_FROM_COMMIT_DISTANCE.
	SteerCapFromCommitDistance bool `mapstructure:"steer_cap_from_commit_distance"`
	// CornerSpeedScale matches CORNER_SPEED_SCALE.
	CornerSpeedScale float64 `mapstructure:"corner_speed_scale"`
	// ReverseSpeedScale matches REVERSE_SPEED_SCALE.
	ReverseSpeedScale float64 `mapstructure:"reverse_speed_scale"`
	// TurnArcHalfFovDeg matches TURN_ARC_HALF_FOV_DEG.
	TurnArcHalfFovDeg float64 `mapstructure:"turn_arc_half_fov_deg"`
	// TurnOpenRangeM matches TURN_OPEN_RANGE_M.
	TurnOpenRangeM float64 `mapstructure:"turn_open_range_m"`
	// CornerLeakMarginM matches CORNER_LEAK_MARGIN_M.
	CornerLeakMarginM float64 `mapstructure:"corner_leak_margin_m"`
	// MinReverseClearanceM matches MIN_REVERSE_CLEARANCE_M.
	MinReverseClearanceM float64 `mapstructure:"min_reverse_clearance_m"`
	// BayWallClearanceM matches BAY_WALL_CLEARANCE_M.
	BayWallClearanceM float64 `mapstructure:"bay_wall_clearance_m"`

	// The in-bay start and bay-exit family. Absent from this mirror until
	// 2026-09-05, and absent from corridor_follower.toml too, so both
	// stacks read their own literals and the two silently drifted apart --
	// Go still had the clearance guard OFF and the arc at 0.3 while Python
	// shipped the solved full-lock ratchet. They are TOML-driven now, so
	// the shipped file is the single source and a drift like that cannot
	// recur without an edit that shows up in a diff.
	//
	// LoadWithDefaults supplies each one's shipped value, so a TOML that
	// omits a key still yields the shipped behaviour rather than a Go zero
	// value -- which for the bools and the arc would silently be the
	// REFUTED configuration.

	// AssumeBayStart matches ASSUME_BAY_START.
	AssumeBayStart bool `mapstructure:"assume_bay_start"`
	// BayExitClearanceGuard matches BAY_EXIT_CLEARANCE_GUARD.
	BayExitClearanceGuard bool `mapstructure:"bay_exit_clearance_guard"`
	// BayExitClearanceMarginM matches BAY_EXIT_CLEARANCE_MARGIN_M.
	BayExitClearanceMarginM float64 `mapstructure:"bay_exit_clearance_margin_m"`
	// BayExitClearanceToleranceM matches BAY_EXIT_CLEARANCE_TOLERANCE_M --
	// see corridorfollower.Config.BayExitClearanceToleranceM.
	BayExitClearanceToleranceM float64 `mapstructure:"bay_exit_clearance_tolerance_m"`
	// BayExitArcSteerNorm matches BAY_EXIT_ARC_STEER_NORM. A cliff at 1.0.
	BayExitArcSteerNorm float64 `mapstructure:"bay_exit_arc_steer_norm"`
	// BayExitSpeedScale matches BAY_EXIT_SPEED_SCALE. A cliff at both ends.
	BayExitSpeedScale float64 `mapstructure:"bay_exit_speed_scale"`
	// BayExitCycle matches BAY_EXIT_CYCLE.
	BayExitCycle bool `mapstructure:"bay_exit_cycle"`
	// BayExitCycleReverseM matches BAY_EXIT_CYCLE_REVERSE_M.
	BayExitCycleReverseM float64 `mapstructure:"bay_exit_cycle_reverse_m"`
	// BayExitCycleReverseSteerNorm matches BAY_EXIT_CYCLE_REVERSE_STEER_NORM.
	BayExitCycleReverseSteerNorm float64 `mapstructure:"bay_exit_cycle_reverse_steer_norm"`
	// BayExitForwardM matches BAY_EXIT_FORWARD_M.
	BayExitForwardM float64 `mapstructure:"bay_exit_forward_m"`
	// BayExitReverseM matches BAY_EXIT_REVERSE_M.
	BayExitReverseM float64 `mapstructure:"bay_exit_reverse_m"`
	// BayExitSteerNorm matches BAY_EXIT_STEER_NORM.
	BayExitSteerNorm float64 `mapstructure:"bay_exit_steer_norm"`
	// BayExitReverseSteerNorm matches BAY_EXIT_REVERSE_STEER_NORM. REFUTED
	// at any non-zero value; 0.0 keeps the refutation recorded.
	BayExitReverseSteerNorm float64 `mapstructure:"bay_exit_reverse_steer_norm"`
	// BayExitHoldSteer matches BAY_EXIT_HOLD_STEER.
	BayExitHoldSteer bool `mapstructure:"bay_exit_hold_steer"`
	// BayExitLegStallTicks matches BAY_EXIT_LEG_STALL_TICKS. 1 is a cliff.
	BayExitLegStallTicks int `mapstructure:"bay_exit_leg_stall_ticks"`
	// BayExitLatchDirection matches BAY_EXIT_LATCH_DIRECTION.
	BayExitLatchDirection bool `mapstructure:"bay_exit_latch_direction"`
	// BayExitLatchReverse matches BAY_EXIT_LATCH_REVERSE.
	BayExitLatchReverse bool `mapstructure:"bay_exit_latch_reverse"`
	// BayExitFallbackFrames matches BAY_EXIT_FALLBACK_FRAMES; 0 = never.
	BayExitFallbackFrames int `mapstructure:"bay_exit_fallback_frames"`
	// BayExitMaxFrames matches BAY_EXIT_MAX_FRAMES; 0 = forever.
	BayExitMaxFrames int `mapstructure:"bay_exit_max_frames"`
	// BayExitGuardOverlapRecovery matches BAY_EXIT_GUARD_OVERLAP_RECOVERY.
	BayExitGuardOverlapRecovery bool `mapstructure:"bay_exit_guard_overlap_recovery"`
	// BayExitOpenSideSectorDeg matches BAY_EXIT_OPEN_SIDE_SECTOR_DEG.
	BayExitOpenSideSectorDeg float64 `mapstructure:"bay_exit_open_side_sector_deg"`
	// BayExitOpenSideVotes matches BAY_EXIT_OPEN_SIDE_VOTES.
	BayExitOpenSideVotes int `mapstructure:"bay_exit_open_side_votes"`
	// BayExitSpeedMPS matches BAY_EXIT_SPEED_MPS.
	BayExitSpeedMPS float64 `mapstructure:"bay_exit_speed_mps"`
	// BayExitContactDistM matches BAY_EXIT_CONTACT_DIST_M.
	BayExitContactDistM float64 `mapstructure:"bay_exit_contact_dist_m"`
	// BayExitContactRecoveryTicks matches BAY_EXIT_CONTACT_RECOVERY_TICKS.
	BayExitContactRecoveryTicks int `mapstructure:"bay_exit_contact_recovery_ticks"`
	// BayExitTargetYawDeg matches BAY_EXIT_TARGET_YAW_DEG.
	BayExitTargetYawDeg float64 `mapstructure:"bay_exit_target_yaw_deg"`
	// BayExitLegMaxS matches BAY_EXIT_LEG_MAX_S.
	BayExitLegMaxS float64 `mapstructure:"bay_exit_leg_max_s"`
	// BayExitGuardBlockTicks matches BAY_EXIT_GUARD_BLOCK_TICKS.
	BayExitGuardBlockTicks int `mapstructure:"bay_exit_guard_block_ticks"`
	// BayExitGuardMeasuredCoast matches BAY_EXIT_GUARD_MEASURED_COAST. SHIPS
	// FALSE, INERT -- no consuming logic reads this; kept for config parity.
	BayExitGuardMeasuredCoast bool `mapstructure:"bay_exit_guard_measured_coast"`
	// BayExitGuardMirrorsReverse matches BAY_EXIT_GUARD_MIRRORS_REVERSE. SHIPS
	// FALSE, INERT -- no consuming logic reads this; kept for config parity.
	BayExitGuardMirrorsReverse bool `mapstructure:"bay_exit_guard_mirrors_reverse"`
	// BayExitDrUsesMeasuredYaw matches BAY_EXIT_DR_USES_MEASURED_YAW. SHIPS
	// FALSE, INERT -- no consuming logic reads this; kept for config parity.
	BayExitDrUsesMeasuredYaw bool `mapstructure:"bay_exit_dr_uses_measured_yaw"`
}

// DefaultCorridorFollowerTOMLPath is
// src/config/navigation/blind_nav/corridor_follower.toml,
// relative to the repo root. No per-component profile overlays -- pass
// nil profileNames to Load.
const DefaultCorridorFollowerTOMLPath = "src/config/navigation/blind_nav/corridor_follower.toml"

// DefaultBayWallClearanceM matches CorridorFollowerParams.BAY_WALL_CLEARANCE_M's
// Pydantic default.
const DefaultBayWallClearanceM = 0.20

// CorridorFollowerDefaults is the LoadWithDefaults defaults map for
// CorridorFollowerConfig.
//
// Every bay key is listed because a Go zero value is not a neutral fallback
// here: `false` for the guard and the latches, and 0.0 for the arc, are all
// REFUTED configurations that measure 0/256 out of the pocket. A TOML missing
// a key must yield the shipped behaviour, not the worst one.
//
// These are literals rather than references to corridorfollower's Default*
// constants because corridorfollower imports this package, not the other way
// round. internal/config/profile's own test asserts the two agree.
func CorridorFollowerDefaults() map[string]any {
	return map[string]any{
		"bay_wall_clearance_m":              DefaultBayWallClearanceM,
		"assume_bay_start":                  true,
		"bay_exit_clearance_guard":          true,
		"bay_exit_clearance_margin_m":       0.005,
		"bay_exit_clearance_tolerance_m":    0.0,
		"bay_exit_arc_steer_norm":           1.0,
		"bay_exit_speed_scale":              0.35,
		"bay_exit_cycle":                    true,
		"bay_exit_cycle_reverse_m":          0.09,
		"bay_exit_cycle_reverse_steer_norm": 0.0,
		"bay_exit_forward_m":                0.08,
		"bay_exit_reverse_m":                0.05,
		"bay_exit_steer_norm":               1.0,
		"bay_exit_reverse_steer_norm":       0.0,
		"bay_exit_hold_steer":               true,
		"bay_exit_leg_stall_ticks":          6,
		"bay_exit_latch_direction":          true,
		"bay_exit_latch_reverse":            false,
		"bay_exit_fallback_frames":          0,
		"bay_exit_max_frames":               0,
		"bay_exit_guard_overlap_recovery":   true,
		"bay_exit_open_side_sector_deg":     15.0,
		"bay_exit_open_side_votes":          5,
		"bay_exit_speed_mps":                0.10,
		"bay_exit_contact_dist_m":           0.08,
		"bay_exit_contact_recovery_ticks":   0,
		"bay_exit_target_yaw_deg":           70.0,
		"bay_exit_leg_max_s":                0.5,
		"bay_exit_guard_block_ticks":        0,
		"bay_exit_guard_measured_coast":     false,
		"bay_exit_guard_mirrors_reverse":    false,
		"bay_exit_dr_uses_measured_yaw":     false,
	}
}
