// Command pi5 is the production combined board binary for the Pi 5: IMU +
// LIDAR + vision + telemetry + state-machine + track-navigator + capture, run as
// supervised goroutines in a single process. See
// platform/robot/docs/internal/plans/go-migration-plan.md ("Process model").
//
// This revision wires the camera capture loop (internal/node/capture) as a
// supervised target so the Pi 5 records video + photos from boot, mirroring the
// Python robot's "record from power-on" behavior. The other subsystems (IMU,
// LIDAR, vision, nav) are added as further supervised targets as they land; this
// is the first real (non-stub) incarnation of cmd/pi5.
package main

import (
	"context"
	"errors"
	"flag"
	"log/slog"
	"os"
	"os/signal"
	"path/filepath"
	"strings"
	"syscall"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/camera"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/capture"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/supervise"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// repoRoot walks up from the working directory to the repo root (the directory
// containing data/), so robot.toml can be located without an absolute path.
func repoRoot() (string, error) {
	dir, err := os.Getwd()
	if err != nil {
		return "", err
	}
	for {
		if info, statErr := os.Stat(filepath.Join(dir, "data")); statErr == nil && info.IsDir() {
			return dir, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return "", errors.New("pi5: repo root (data/) not found walking up from cwd")
		}
		dir = parent
	}
}

type cliConfig struct {
	natsURL    string
	nodeName   string
	runsRoot   string
	configRoot string
	profiles   string
	fps        float64
	video      bool
	photoEvery time.Duration
}

func main() {
	os.Exit(runMain())
}

func runMain() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	var cfg cliConfig
	fs := flag.NewFlagSet("pi5", flag.ContinueOnError)
	fs.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	fs.StringVar(&cfg.nodeName, "name", "pi5", "NATS client name")
	fs.StringVar(&cfg.runsRoot, "runs-root", "", "runs root dir (default: repo-root data/runs_pulled)")
	fs.StringVar(&cfg.configRoot, "config-root", "", "repo root for robot.toml (VTITAN_HARDWARE_PROFILE selects the active profile)")
	fs.StringVar(&cfg.profiles, "profiles", "", "comma-separated hardware profiles (overrides VTITAN_HARDWARE_PROFILE)")
	fs.Float64Var(&cfg.fps, "fps", 15.0, "capture frame rate")
	fs.BoolVar(&cfg.video, "video", true, "record the debug video")
	fs.DurationVar(&cfg.photoEvery, "photo-interval", 10*time.Second, "periodic dataset-photo cadence (0 = off)")
	if err := fs.Parse(os.Args[1:]); err != nil {
		return 1
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	camCfg, err := loadCamera(ctx, cfg, logger)
	if err != nil {
		logger.Error("pi5: loading camera config", "error", err)
		return 1
	}

	supervisor, err := supervise.New(supervise.DefaultConfig(), logger)
	if err != nil {
		logger.Error("pi5: creating supervisor", "error", err)
		return 1
	}

	// Capture is the first supervised target; IMU/LIDAR/vision/nav join as they
	// are ported. Each runs as its own supervised goroutine, sharing one process
	// and (where relevant) one NATS connection.
	captureCfg := capture.Config{
		Camera:          camCfg,
		NATS:            nats.DefaultConfig(cfg.natsURL, cfg.nodeName),
		RunsRoot:        cfg.runsRoot,
		FPS:             cfg.fps,
		Video:           cfg.video,
		PhotoInterval:   cfg.photoEvery,
		PhotoSubdir:     "captures",
		PhotoRequireDet: false,
	}
	if err = supervisor.RunAll(ctx,
		supervise.Target{Name: "capture", Fn: func(ctx context.Context) error {
			return capture.Run(ctx, captureCfg, logger)
		}},
	); err != nil {
		if errors.Is(err, context.Canceled) {
			return 0
		}
		logger.Error("pi5: supervisor exited", "error", err)
		return 1
	}
	return 0
}

// loadCamera reads robot.toml's [camera] section and maps it to a camera.Config.
// It falls back to a synthetic source when no hardware profile is configured, so
// cmd/pi5 is runnable on a dev machine without a CSI camera.
func loadCamera(_ context.Context, cfg cliConfig, logger *slog.Logger) (camera.Config, error) {
	profiles := splitProfiles(cfg.profiles)
	basePath := cfg.configRoot
	if basePath == "" {
		root, err := repoRoot()
		if err != nil {
			logger.Warn("pi5: repo root not found, defaulting to synthetic camera", "error", err)
			return camera.Config{Source: camera.SourceSynthetic, FPS: cfg.fps}, nil
		}
		basePath = filepath.Join(root, profile.DefaultRobotTOMLPath)
	}
	rc, err := profile.LoadRobotConfig(basePath, profiles)
	if err != nil {
		logger.Warn("pi5: no robot config loaded, defaulting to synthetic camera", "error", err)
		return camera.Config{Source: camera.SourceSynthetic, FPS: cfg.fps}, nil
	}
	cam := rc.Camera
	source := cam.Source
	if source == "" {
		source = camera.SourceSynthetic
	}
	return camera.Config{
		Source:      source,
		Device:      cam.Device,
		Width:       cam.Width,
		Height:      cam.Height,
		FPS:         cfg.fps,
		NATSSubject: "vtitan.sensor.v1.camera",
	}, nil
}

func splitProfiles(s string) []string {
	if s == "" {
		return nil
	}
	var out []string
	for _, p := range strings.Split(s, ",") {
		if p = strings.TrimSpace(p); p != "" {
			out = append(out, p)
		}
	}
	return out
}
