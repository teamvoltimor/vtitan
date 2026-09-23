package profile

import (
	"fmt"
	"log/slog"
	"os"
	"path/filepath"

	"github.com/spf13/viper"
)

// Apply loads T from path (with the profileNames overlays) and, on success,
// hands the decoded value to apply. On failure it logs a warning and returns
// false, leaving the caller's defaults in place, so a component degrades to its
// shipped literals rather than to a half-loaded config.
//
// This is the shared form of the load-warn-apply block every component's
// ConfigFor used to repeat. Each source is loaded independently because they
// are unrelated failure domains: one missing file must not discard the rest.
func Apply[T any](
	logger *slog.Logger,
	path string,
	profileNames []string,
	apply func(T),
) bool {
	loaded, err := Load[T](path, profileNames)
	if err != nil {
		logger.Warn("config: loading TOML, falling back to defaults",
			"path", path, "error", err)
		return false
	}
	apply(*loaded)
	return true
}

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
// A missing base file is an error. A profileNames entry that no profiles
// tree knows is an error, matching
// shared.config.hardware_profile.profile_dirs(): not basePath's own
// profiles/<name>, nor that of any directory above it (src/config/profiles
// sits above every component tree). A name the component's own tree lacks
// but an enclosing one knows is skipped, as is a profile directory with no
// file named like basePath's, matching settings_base.py's per-driver overlay
// behavior: not every profile touches every component, and a servo profile
// must not stop the LIDAR's file from loading.
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
			if !knownAbove(dir, name) {
				return nil, fmt.Errorf("profile: %q has no directory %s, nor in any profiles tree above it",
					name, profileDir)
			}
			continue
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

// knownAbove reports whether a directory strictly above dir holds a
// profiles/<name> directory, i.e. whether name is a real profile that simply
// does not touch the component living in dir.
func knownAbove(dir, name string) bool {
	for parent := filepath.Dir(dir); parent != dir; dir, parent = parent, filepath.Dir(parent) {
		if info, err := os.Stat(filepath.Join(parent, "profiles", name)); err == nil && info.IsDir() {
			return true
		}
	}
	return false
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
