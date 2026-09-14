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
// (viper.MergeInConfig deep-merges nested tables). defaults, when non-nil, is
// applied via viper.SetDefault before reading -- the caller's explicit
// fallbacks for keys an overlay deliberately omits, not shipped per-type
// values: every config value lives in the TOML, which is complete for each
// file it describes and validated against its schema by Taplo.
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
//
// Every value comes from the TOML: there is no per-type shipped-default layer,
// so a key absent from every source file reads as the Go zero value. The
// shipped files carry every key their schema declares, which Taplo enforces at
// lint time, so a zero at runtime means a hand-edited or partial source, not a
// missing default.
func Load[T any](basePath string, profileNames []string) (*T, error) {
	return load[T](basePath, profileNames, nil)
}

// LoadWithDefaults is Load, but also applies the given defaults (dotted TOML
// key -> value, e.g. "min_mps") via viper.SetDefault before reading -- for a
// caller that is re-overlaying an already-resolved config and wants the
// overlay to leave every key it does not itself set at the resolved value
// (see internal/nav/navigator.loadSpeedConfig). Those defaults are the
// caller's own resolved values, not a second copy of the shipped config.
func LoadWithDefaults[T any](
	basePath string,
	profileNames []string,
	defaults map[string]any,
) (*T, error) {
	return load[T](basePath, profileNames, defaults)
}

// load is the shared tail of Load and LoadWithDefaults: merge the sources,
// then decode into a new T.
func load[T any](basePath string, profileNames []string, defaults map[string]any) (*T, error) {
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
