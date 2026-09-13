// Package simconfigtest provides test helpers for loading the real track.toml
// that the sim pipeline works from.
package simconfigtest

import (
	"errors"
	"os"
	"path/filepath"
	"runtime"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// Load loads the repository's src/config/track.toml, locating the config root
// by walking up from this file to the directory holding go.mod. It uses the
// robot loaded by LoadRobot for the chassis-width spawn offsets.
func Load(tb testing.TB) *simconfig.Track {
	tb.Helper()
	root, err := configRoot()
	if err != nil {
		tb.Fatalf("simconfigtest: %v", err)
	}
	track, err := simconfig.LoadTrack(root, LoadRobot(tb).RobotWidth)
	if err != nil {
		tb.Fatalf("simconfigtest: load track: %v", err)
	}
	return track
}

// LoadRobot loads the repository's src/config/robot.toml overlaid with the
// shipped hardware-profile pair, so tests need no VTITAN_HARDWARE_PROFILE.
func LoadRobot(tb testing.TB) *simconfig.Robot {
	tb.Helper()
	root, err := configRoot()
	if err != nil {
		tb.Fatalf("simconfigtest: %v", err)
	}
	robot, err := simconfig.LoadRobot(root, simconfig.DefaultHardwareProfiles())
	if err != nil {
		tb.Fatalf("simconfigtest: load robot: %v", err)
	}
	return robot
}

// configRoot resolves <repo>/src/config from this file's location.
func configRoot() (string, error) {
	_, file, _, ok := runtime.Caller(0)
	if !ok {
		return "", errors.New("cannot resolve test helper source location")
	}
	dir := filepath.Dir(file)
	for {
		if _, err := os.Stat(filepath.Join(dir, "go.mod")); err == nil {
			return filepath.Join(dir, "..", "config"), nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return "", errors.New("go.mod not found above the test helper")
		}
		dir = parent
	}
}
