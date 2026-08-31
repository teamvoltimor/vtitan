package waypoints

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
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
	// CORNER_ARC_ASSUME_WIDE defaults to True in the Python model and is
	// absent from the checked-in waypoints.toml, so it must be defaulted
	// here rather than silently read as false -- the same pattern as
	// controllers' min_history_for_distance (see EscapeConfig) and
	// CorridorFollowerConfig's bay_wall_clearance_m.
	waypointDefaults := map[string]any{"corner_arc_assume_wide": DefaultCornerArcAssumeWide}
	loaded, err := profile.LoadWithDefaults[profile.WaypointsConfig](basePath, nil, waypointDefaults)
	if err != nil {
		logger.Warn("waypoints: loading waypoints.toml, falling back to defaults",
			"config_root", configRoot, "error", err)
		return cfg
	}

	cfg.DedupeDistanceM = loaded.DedupeDistanceM
	cfg.WideCenterBiasM = loaded.WideCenterBiasM
	cfg.NarrowCenterBiasM = loaded.NarrowCenterBiasM
	cfg.NarrowWidthThresholdM = loaded.NarrowWidthThresholdM
	cfg.NumIntermediateArcPoints = loaded.NumIntermediateArcPoints
	cfg.StraightWaypointCount = loaded.StraightWaypointCount
	cfg.ArcRadius = loaded.ArcRadius
	cfg.CornerArcAssumeWide = loaded.CornerArcAssumeWide

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
