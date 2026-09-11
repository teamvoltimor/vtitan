// Package recording owns the on-disk layout of a robot run: the run directory
// is the unit of capture, holding the MCAP bag, the debug video, and the
// periodic dataset photos. Both the Go and Python stacks write into the same
// repo-root data/ tree (see repo-root .gitignore), never into a module-local
// dir, so either side can read what the other recorded.
//
// Only RunRoot is defined here so far; RunRecorder/VideoWriter/PhotoCapture
// land in later change sets.
package recording

import (
	"fmt"
	"os"
	"path/filepath"
)

// LiveDir and SimDir are the two provenance roots under the repo-root data/
// directory: artifacts pulled off the robot vs. artifacts recorded by the
// simulator. Kept apart on purpose: a sim sweep can emit hundreds of bags in
// seconds, and mixing them into the hardware tree would bury the handful of
// real track runs that tree exists to hold -- and those are expensive to
// re-record. The bags themselves are the same format, so a sim bag opens in
// Foxglove Studio and replays through test/bagreplay exactly like a hardware
// one.
const (
	LiveDir = "live"
	SimDir  = "sim"
)

// RunsDir, VideosDir, PhotosDir are the three artifact subtrees nested under
// either LiveDir or SimDir. Large binary data; gitignored, .gitkeep kept.
const (
	RunsDir   = "runs"
	VideosDir = "videos"
	PhotosDir = "photos"
)

// dataDirName is the repo-root directory that holds every pulled artifact.
const dataDirName = "data"

// dirMode is the permission mode for artifact directories: owner read/write/
// execute, group and others read/execute, matching the pulled hardware tree.
const dirMode = 0o750

// RunRoot returns the absolute path to <repo-root>/data, the shared pulled-
// artifact tree. It walks up from the current working directory (the module is
// always built/run beneath the repo root) until it finds a directory containing
// data/, so it works whether invoked from the module, the repo root, or a test
// binary elsewhere under the tree. It does not hardcode an absolute path.
func RunRoot() (string, error) {
	dir, err := filepath.Abs(".")
	if err != nil {
		return "", fmt.Errorf("recording: resolving cwd: %w", err)
	}
	for {
		candidate := filepath.Join(dir, dataDirName)
		if info, statErr := statDir(candidate); statErr == nil && info {
			return candidate, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return "", fmt.Errorf("recording: no %q directory found walking up from cwd", dataDirName)
		}
		dir = parent
	}
}

// RunsRoot is RunRoot joined with LiveDir/RunsDir (data/live/runs).
func RunsRoot() (string, error) {
	root, err := RunRoot()
	if err != nil {
		return "", err
	}
	return filepath.Join(root, LiveDir, RunsDir), nil
}

// SimRunsRoot is RunRoot joined with SimDir/RunsDir (data/sim/runs).
func SimRunsRoot() (string, error) {
	root, err := RunRoot()
	if err != nil {
		return "", err
	}
	return filepath.Join(root, SimDir, RunsDir), nil
}

// statDir reports whether path exists and is a directory.
func statDir(path string) (bool, error) {
	info, err := os.Stat(path)
	if err != nil {
		if os.IsNotExist(err) {
			return false, nil
		}
		return false, fmt.Errorf("recording: stating %s: %w", path, err)
	}
	return info.IsDir(), nil
}
