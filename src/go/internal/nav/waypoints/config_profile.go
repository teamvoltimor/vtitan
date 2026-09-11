package waypoints

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with profile.WaypointsConfig (loaded from
// <configRoot>/profile.DefaultWaypointsTOMLPath) if configRoot is
// non-empty and loading succeeds; otherwise, or on load failure, the
// literal defaults, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath)
	// All three shipped defaults (corner_arc_assume_wide,
	// unconfirmed_width_inner_bias_m, defer_current_corridor_replan) live on
	// WaypointsConfig as `default` tags, so profile.Load applies them and an
	// absent key reads as the shipped value rather than silently as false /
	// 0.0. A bool and a bias both revert SILENTLY, which is off rather than
	// absent, and disabling assume-wide corner sizing or both halves of the
	// 596 -> 638/640 Open result would look like a clean load. Same pattern
	// as controllers' min_history_for_distance (see EscapeConfig) and
	// CorridorFollowerConfig's bay_wall_clearance_m.
	loaded, err := profile.Load[profile.WaypointsConfig](basePath, nil)
	if err != nil {
		logger.Warn("waypoints: loading waypoints.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.DedupeDistanceM = loaded.DedupeDistanceM
	cfg.WideCenterBiasM = loaded.WideCenterBiasM
	cfg.NarrowCenterBiasM = loaded.NarrowCenterBiasM
	cfg.ObstaclesCenterBiasM = loaded.ObstaclesCenterBiasM
	cfg.NarrowWidthThresholdM = loaded.NarrowWidthThresholdM
	cfg.NumIntermediateArcPoints = loaded.NumIntermediateArcPoints
	cfg.StraightWaypointCount = loaded.StraightWaypointCount
	cfg.ArcRadius = loaded.ArcRadius
	cfg.CornerArcAssumeWide = loaded.CornerArcAssumeWide
	cfg.UnconfirmedWidthInnerBiasM = loaded.UnconfirmedWidthInnerBiasM
	cfg.DeferCurrentCorridorReplan = loaded.DeferCurrentCorridorReplan

	if side, ok := corridorSideFromString(loaded.WideCenterBiasSide); ok {
		cfg.WideCenterBiasSide = side
	} else {
		logger.Warn("waypoints: waypoints.toml's wide_center_bias_side is not \"inner\"/\"outer\", keeping default",
			"value", loaded.WideCenterBiasSide)
	}
	if side, ok := corridorSideFromString(loaded.NarrowCenterBiasSide); ok {
		cfg.NarrowCenterBiasSide = side
	} else {
		logger.Warn("waypoints: waypoints.toml's narrow_center_bias_side is not \"inner\"/\"outer\", keeping default",
			"value", loaded.NarrowCenterBiasSide)
	}

	return cfg
}

// corridorSideFromString parses waypoints.toml's *_center_bias_side
// string values, matching shared.domain.enums.CorridorSide's
// "inner"/"outer" wire values.
func corridorSideFromString(s string) (side trackmodel.CorridorSide, ok bool) {
	switch s {
	case "inner":
		return trackmodel.Inner, true
	case "outer":
		return trackmodel.Outer, true
	default:
		return 0, false
	}
}
