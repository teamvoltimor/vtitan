package profile_test

import (
	"math"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
)

func TestLoad_DirectionEstimatorConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.DirectionEstimatorConfig](
		filepath.Join("testdata", "direction_estimator.toml"), nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if math.Abs(cfg.AlignmentToleranceRad-0.4363323129985824) > 1e-9 {
		t.Errorf("AlignmentToleranceRad = %v, want ~0.4363", cfg.AlignmentToleranceRad)
	}
	if cfg.MinVotes != 5 {
		t.Errorf("MinVotes = %v, want 5", cfg.MinVotes)
	}
	if cfg.MaxInTrackRangeM != 4.5 || cfg.MinAsymmetryM != 0.20 ||
		cfg.PlausibleSpanThresholdM != 1.25 {
		t.Errorf("unexpected field values: %+v", cfg)
	}
}

func TestLoad_CorridorFollowerConfig_UsesDefaultForOmittedField(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.CorridorFollowerConfig](
		filepath.Join(
			"testdata",
			"corridor_follower.toml",
		),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.MinForwardClearanceM != 0.30 || cfg.TurnClearanceM != 0.60 {
		t.Errorf("TOML-present fields wrong: %+v", cfg)
	}
	if cfg.BayWallClearanceM != profile.DefaultBayWallClearanceM {
		t.Errorf("BayWallClearanceM = %v, want default %v (field omitted from testdata TOML)",
			cfg.BayWallClearanceM, profile.DefaultBayWallClearanceM)
	}
}

func TestLoad_LidarSectorsConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.LidarSectorsConfig](
		filepath.Join("testdata", "lidar_sectors.toml"),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.DirectionArcHalfFovDeg != 8.0 || cfg.MinValidRangeM != 0.05 {
		t.Errorf("unexpected field values: %+v", cfg)
	}
}

func TestLoad_WaypointsConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.WaypointsConfig](
		filepath.Join("testdata", "waypoints.toml"),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.DedupeDistanceM != 0.001 {
		t.Errorf("DedupeDistanceM = %v, want 0.001", cfg.DedupeDistanceM)
	}
	if cfg.WideCenterBiasM != 0.10 || cfg.WideCenterBiasSide != "inner" {
		t.Errorf("wide bias = %v/%q, want 0.10/inner", cfg.WideCenterBiasM, cfg.WideCenterBiasSide)
	}
	if cfg.NarrowCenterBiasM != 0.0 || cfg.NarrowCenterBiasSide != "inner" {
		t.Errorf(
			"narrow bias = %v/%q, want 0.0/inner",
			cfg.NarrowCenterBiasM,
			cfg.NarrowCenterBiasSide,
		)
	}
	if cfg.NarrowWidthThresholdM != 0.8 {
		t.Errorf("NarrowWidthThresholdM = %v, want 0.8", cfg.NarrowWidthThresholdM)
	}
	if cfg.NumIntermediateArcPoints != 3 || cfg.StraightWaypointCount != 8 {
		t.Errorf(
			"counts = %v/%v, want 3/8",
			cfg.NumIntermediateArcPoints,
			cfg.StraightWaypointCount,
		)
	}
}
