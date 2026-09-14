package profile

import (
	"fmt"
	"reflect"
	"strconv"
	"strings"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/blind_nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/escape"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/sensors"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/signs"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/waypoint"
)

// configDefaultsByType maps a generated config DTO to the shipped fallbacks
// its schema does not carry. The generated structs have no `default` struct
// tags, so those fallbacks cannot live on the type the way they do for the
// hand-written configs; keying a registry by the decoded type keeps them
// available to configDefaults without wrapping or aliasing the DTO.
//
// Every value here restates the exact literal the previous companion
// `*Defaults` struct carried. A type absent from this map has no fallbacks and
// falls through to tagDefaults, which reads whatever `default` tags its
// hand-written struct declares (or none, for an untagged generated DTO).
var configDefaultsByType = map[reflect.Type]map[string]any{
	reflect.TypeFor[generated.CompetitionSpecs](): {
		"round_time_limit_s":      180.0,
		"open_challenge_laps":     3,
		"obstacle_challenge_laps": 3,
	},
	reflect.TypeFor[blind_nav.NavigationBlindNavCorridorFollower](): {
		"bay_wall_clearance_m":            0.20,
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
	},
	reflect.TypeFor[escape.NavigationEscapeEscape](): {
		"min_history_for_distance": 2,
	},
	reflect.TypeFor[sensors.NavigationSensorsLidarSectors](): {
		"rear_self_detection_from_chassis": true,
	},
	reflect.TypeFor[signs.NavigationSignsSignDiscovery](): {
		"min_reliable_bbox_height_px": 5,
		"max_ingest_range_m":          2.0,
		"association_dist_m":          0.25,
		"min_hits":                    3,
		"robot_corridor_flip_ticks":   5,
	},
	reflect.TypeFor[signs.NavigationSignsSignRouter](): {
		"depth_pin":                           true,
		"pin_corner_guard":                    true,
		"pin_heading_guard":                   true,
		"pin_heading_guard_deg":               35.0,
		"sign_lane_relabel_unsatisfiable":     true,
		"sign_lane_depth_consistent_corridor": true,
		"slot_sign_map":                       false,
		"slot_accept_radius_m":                0.30,
		"slot_min_evidence":                   0.75,
		"slot_repoint_margin":                 1.5,
	},
	reflect.TypeFor[waypoint.NavigationWaypointWaypoints](): {
		"corner_arc_assume_wide":         true,
		"unconfirmed_width_inner_bias_m": 0.05,
		"defer_current_corridor_replan":  true,
	},
}

// tagDefaults reflect-walks T and builds the viper.SetDefault map from each
// field's `default:"..."` struct tag, keyed by its `mapstructure` name and
// dotted for nested structs.
//
// This is what lets a TOML key some source files never set fall back to the
// shipped value without a parallel map[string]any: the default sits on the
// field it belongs to, beside the same `mapstructure` tag mapstructure decodes
// by, so the two cannot drift. A field with no `default` tag contributes
// nothing and an omitted key reads as the Go zero value, exactly as before.
func tagDefaults[T any]() (map[string]any, error) {
	defaults := map[string]any{}
	if err := collectDefaults(reflect.TypeFor[T](), "", defaults); err != nil {
		return nil, err
	}
	return defaults, nil
}

// collectDefaults walks value type t under the dotted key prefix, recording
// each tagged scalar in out. Unexported fields and `mapstructure:"-"` are
// skipped, matching how viper/mapstructure address the decoded struct; a
// pointer's pointee is what carries the kind to parse.
func collectDefaults(t reflect.Type, prefix string, out map[string]any) error {
	for field := range t.Fields() {
		if !field.IsExported() {
			continue
		}
		name := mapstructureName(field)
		if name == "" {
			continue
		}
		key := name
		if prefix != "" {
			key = prefix + "." + name
		}
		if err := collectField(field, key, out); err != nil {
			return err
		}
	}
	return nil
}

// collectField records the `default` tag of one field under key, or recurses
// into it when it is an untagged struct (a nested TOML table).
func collectField(field reflect.StructField, key string, out map[string]any) error {
	valueType := field.Type
	for valueType.Kind() == reflect.Pointer {
		valueType = valueType.Elem()
	}

	raw := field.Tag.Get("default")
	if raw == "" {
		if valueType.Kind() == reflect.Struct {
			return collectDefaults(valueType, key, out)
		}
		return nil
	}

	value, err := parseDefault(valueType, raw)
	if err != nil {
		return fmt.Errorf("profile: %s: %w", key, err)
	}
	out[key] = value
	return nil
}

// mapstructureName returns the TOML key a field decodes from: the name half of
// its `mapstructure` tag, or the lowercased field name (viper's own
// case-insensitive fallback) when that tag carries no name. An explicit "-"
// means the field is not addressable and yields "".
func mapstructureName(field reflect.StructField) string {
	tag := field.Tag.Get("mapstructure")
	if tag == "-" {
		return ""
	}
	if name, _, _ := strings.Cut(tag, ","); name != "" {
		return name
	}
	return strings.ToLower(field.Name)
}

// parseDefault parses a `default` tag into the field's own type, so the value
// viper stores has the exact kind mapstructure will assign rather than a
// generic int64/float64 viper would have to coerce back.
func parseDefault(valueType reflect.Type, raw string) (any, error) {
	value := reflect.New(valueType).Elem()
	switch valueType.Kind() {
	case reflect.Bool:
		parsed, err := strconv.ParseBool(raw)
		if err != nil {
			return nil, fmt.Errorf("parsing bool default %q: %w", raw, err)
		}
		value.SetBool(parsed)
	case reflect.String:
		value.SetString(raw)
	case reflect.Int, reflect.Int8, reflect.Int16, reflect.Int32, reflect.Int64:
		parsed, err := strconv.ParseInt(raw, 10, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing int default %q: %w", raw, err)
		}
		value.SetInt(parsed)
	case reflect.Uint, reflect.Uint8, reflect.Uint16, reflect.Uint32, reflect.Uint64:
		parsed, err := strconv.ParseUint(raw, 10, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing uint default %q: %w", raw, err)
		}
		value.SetUint(parsed)
	case reflect.Float32, reflect.Float64:
		parsed, err := strconv.ParseFloat(raw, valueType.Bits())
		if err != nil {
			return nil, fmt.Errorf("parsing float default %q: %w", raw, err)
		}
		value.SetFloat(parsed)
	default:
		return nil, fmt.Errorf("unsupported default type %s", valueType)
	}
	return value.Interface(), nil
}
