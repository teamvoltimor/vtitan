package waypoints

import (
	"log/slog"
	"path/filepath"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/waypoint"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// ConfigFor resolves the Config to run with: DefaultConfig's literals,
// overlaid with waypoint.NavigationWaypointWaypoints (loaded from
// <configRoot>/profile.DefaultWaypointsTOMLPath) if configRoot is
// non-empty and loading succeeds; otherwise, or on load failure, the
// literal defaults, logging why.
func ConfigFor(logger *slog.Logger, configRoot string) Config {
	cfg := DefaultConfig()
	if configRoot == "" {
		return cfg
	}

	basePath := filepath.Join(configRoot, profile.DefaultWaypointsTOMLPath)
	// Every value lives in waypoints.toml, so an absent key reads as the Go
	// zero value rather than a shipped fallback. The shipped file carries
	// corner_arc_assume_wide / unconfirmed_width_inner_bias_m /
	// defer_current_corridor_replan, which Taplo checks against the schema at
	// lint time, so a zero at runtime means a hand-edited source.
	profile.Apply(logger, basePath, nil, func(loaded waypoint.NavigationWaypointWaypoints) {
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
			logger.Warn(
				"waypoints: waypoints.toml's narrow_center_bias_side is not \"inner\"/\"outer\", keeping default",
				"value", loaded.NarrowCenterBiasSide,
			)
		}
	})

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
