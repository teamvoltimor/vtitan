package profile

import (
	"path/filepath"
	"reflect"
	"testing"
)

func TestTagDefaults_ParsesEachKind(t *testing.T) {
	t.Parallel()

	type nested struct {
		Flag bool `mapstructure:"flag" default:"true"`
	}
	type sample struct {
		Count  int     `mapstructure:"count"  default:"7"`
		Scale  float64 `mapstructure:"scale"  default:"0.5"`
		Label  string  `mapstructure:"label"  default:"x"`
		Nested nested  `mapstructure:"nested"`
		Skip   string  `mapstructure:"-"`
	}

	got, err := tagDefaults[sample]()
	if err != nil {
		t.Fatalf("tagDefaults: %v", err)
	}

	want := map[string]any{
		"count":       7,
		"scale":       0.5,
		"label":       "x",
		"nested.flag": true,
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("tagDefaults = %#v, want %#v", got, want)
	}
}

func TestTagDefaults_RejectsUnparsableTag(t *testing.T) {
	t.Parallel()

	type sample struct {
		Count int `mapstructure:"count" default:"not-a-number"`
	}

	if _, err := tagDefaults[sample](); err == nil {
		t.Fatal("tagDefaults: want error for unparsable default, got nil")
	}
}

func TestCompetitionConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	got, err := tagDefaults[CompetitionConfig]()
	if err != nil {
		t.Fatalf("tagDefaults: %v", err)
	}

	want := map[string]any{
		"round_time_limit_s":      180.0,
		"open_challenge_laps":     3,
		"obstacle_challenge_laps": 3,
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("tagDefaults = %#v, want %#v", got, want)
	}
	if got["round_time_limit_s"] != DefaultRoundTimeLimitS {
		t.Errorf("round_time_limit_s tag = %v, want DefaultRoundTimeLimitS %v",
			got["round_time_limit_s"], DefaultRoundTimeLimitS)
	}
}

func TestLoad_AppliesTagDefaultsForOmittedKeys(t *testing.T) {
	t.Parallel()

	cfg, err := Load[CompetitionConfig](
		filepath.Join("testdata", "competition_partial.toml"), nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.RoundTimeLimitS != 200.0 {
		t.Errorf("RoundTimeLimitS = %v, want 200.0 (present in TOML)", cfg.RoundTimeLimitS)
	}
	if cfg.OpenChallengeLaps != 3 {
		t.Errorf("OpenChallengeLaps = %v, want 3 (tag default)", cfg.OpenChallengeLaps)
	}
	if cfg.ObstacleChallengeLaps != 3 {
		t.Errorf("ObstacleChallengeLaps = %v, want 3 (tag default)", cfg.ObstacleChallengeLaps)
	}
}

// checkTagDefaults locks a config struct's `default` tags against the shipped
// values, so a later edit to a field's tag cannot silently change what an
// omitted TOML key resolves to.
func checkTagDefaults[T any](t *testing.T, want map[string]any) {
	t.Helper()

	got, err := tagDefaults[T]()
	if err != nil {
		t.Fatalf("tagDefaults: %v", err)
	}
	if !reflect.DeepEqual(got, want) {
		t.Errorf("tagDefaults = %#v, want %#v", got, want)
	}
}

func TestSignRouterConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[SignRouterConfig](t, map[string]any{
		"depth_pin":                           DefaultDepthPin,
		"pin_corner_guard":                    DefaultPinCornerGuard,
		"pin_heading_guard":                   DefaultPinHeadingGuard,
		"pin_heading_guard_deg":               DefaultPinHeadingGuardDeg,
		"sign_lane_relabel_unsatisfiable":     DefaultRelabelUnsatisfiable,
		"sign_lane_depth_consistent_corridor": DefaultDepthConsistentCorridor,
	})
}

func TestSignDiscoveryConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[SignDiscoveryConfig](t, map[string]any{
		"min_reliable_bbox_height_px": 5.0,
		"max_ingest_range_m":          2.0,
		"association_dist_m":          0.25,
		"min_hits":                    3,
		"robot_corridor_flip_ticks":   5,
	})
}

func TestWaypointsConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[WaypointsConfig](t, map[string]any{
		"corner_arc_assume_wide":         true,
		"unconfirmed_width_inner_bias_m": 0.05,
		"defer_current_corridor_replan":  true,
	})
}

func TestCorridorFollowerConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[CorridorFollowerConfig](t, map[string]any{
		"bay_wall_clearance_m":            DefaultBayWallClearanceM,
		"assume_bay_start":                true,
		"bay_exit_clearance_guard":        true,
		"bay_exit_clearance_margin_m":     0.005,
		"bay_exit_arc_steer_norm":         1.0,
		"bay_exit_speed_scale":            0.35,
		"bay_exit_cycle":                  true,
		"bay_exit_cycle_reverse_m":        0.09,
		"bay_exit_forward_m":              0.08,
		"bay_exit_reverse_m":              0.05,
		"bay_exit_steer_norm":             1.0,
		"bay_exit_hold_steer":             true,
		"bay_exit_leg_stall_ticks":        6,
		"bay_exit_latch_direction":        true,
		"bay_exit_guard_overlap_recovery": true,
		"bay_exit_open_side_sector_deg":   15.0,
		"bay_exit_open_side_votes":        5,
		"bay_exit_speed_mps":              0.10,
		"bay_exit_contact_dist_m":         0.08,
		"bay_exit_target_yaw_deg":         70.0,
		"bay_exit_leg_max_s":              0.5,
	})
}

func TestClearanceConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[ClearanceConfig](t, map[string]any{
		"forward_no_data_is_degraded": true,
	})
}

func TestLidarSectorsConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[LidarSectorsConfig](t, map[string]any{
		"rear_self_detection_from_chassis": true,
	})
}

func TestEscapeConfig_DefaultsMatchTags(t *testing.T) {
	t.Parallel()

	checkTagDefaults[EscapeConfig](t, map[string]any{
		"min_history_for_distance": 2,
	})
}
