package profile

import (
	"fmt"
	"os"
	"path/filepath"

	"github.com/spf13/viper"
)

// merge reads basePath as TOML via viper, then for each name in
// profileNames merges that profile's overlay
// (filepath.Join(filepath.Dir(basePath), "profiles", name,
// filepath.Base(basePath))) on top, in order, later names winning
// (viper.MergeInConfig deep-merges nested tables). defaults, if non-nil,
// is applied via viper.SetDefault before reading -- the idiomatic viper
// equivalent of a Pydantic Field(default=...) for a TOML key some source
// files genuinely never set (e.g. a value some but not all navigation-
// tuning TOML files omit, relying on the Python model's own default).
//
// A missing base file is an error. A profileNames entry whose directory
// doesn't exist is an error, matching
// shared.config.hardware_profile.profile_dirs(). A profile directory that
// exists but has no file named like basePath's is skipped rather than an
// error, matching settings_base.py's per-driver overlay behavior: not every
// profile touches every TOML file.
func merge(basePath string, profileNames []string, defaults map[string]any) (*viper.Viper, error) {
	v := viper.New()
	for key, value := range defaults {
		v.SetDefault(key, value)
	}

	v.SetConfigFile(basePath)
	if err := v.ReadInConfig(); err != nil {
		return nil, fmt.Errorf("profile: reading base %s: %w", basePath, err)
	}

	dir, file := filepath.Dir(basePath), filepath.Base(basePath)
	for _, name := range profileNames {
		profileDir := filepath.Join(dir, "profiles", name)
		if _, statErr := os.Stat(profileDir); statErr != nil {
			return nil, fmt.Errorf("profile: %q has no directory %s", name, profileDir)
		}

		overlayPath := filepath.Join(profileDir, file)
		if _, statErr := os.Stat(overlayPath); statErr != nil {
			continue
		}

		v.SetConfigFile(overlayPath)
		if err := v.MergeInConfig(); err != nil {
			return nil, fmt.Errorf("profile: reading overlay for %q: %w", name, err)
		}
	}
	return v, nil
}

// Load reads basePath as TOML, merges profileNames' overlays on top (see
// merge), and unmarshals the result into a new T. Struct fields need a
// `mapstructure:"..."` tag wherever the TOML key isn't just the field name
// lowercased (viper matches case-insensitively but does not
// snake_case-convert).
func Load[T any](basePath string, profileNames []string) (*T, error) {
	return LoadWithDefaults[T](basePath, profileNames, nil)
}

// LoadWithDefaults is Load, but applies defaults (dotted TOML key ->
// value, e.g. "corridor_follower.bay_wall_clearance_m") via
// viper.SetDefault before reading -- for a TOML key some source files
// never set, relying on the Python model's own Field(default=...) instead
// (see merge's doc comment).
func LoadWithDefaults[T any](basePath string, profileNames []string, defaults map[string]any) (*T, error) {
	v, err := merge(basePath, profileNames, defaults)
	if err != nil {
		return nil, err
	}

	var cfg T
	if unmarshalErr := v.Unmarshal(&cfg); unmarshalErr != nil {
		return nil, fmt.Errorf("profile: unmarshaling merged config: %w", unmarshalErr)
	}
	return &cfg, nil
}
