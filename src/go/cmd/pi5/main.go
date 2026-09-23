// Command pi5 is the production combined board binary for the Pi 5: IMU +
// LIDAR + vision + telemetry + state-machine + track-navigator + capture, run as
// supervised goroutines in a single process. See
// adr:0068-go-parallel-track-single-cutover ("Process model").
//
// This revision wires two supervised targets: the camera capture loop
// (internal/node/capture), so the Pi 5 records video + photos from boot
// mirroring the Python robot's "record from power-on" behavior, plus the IMU
// and LIDAR publish loops (internal/node/imu, internal/node/lidar) -- the same
// loops cmd/imu-node and cmd/lidar-node run, the telemetry-summary aggregation
// that feeds cmd/pi-zero's OLED (internal/node/telemetry), and -- only when a
// --robot-id is given -- the backend command channel
// (internal/node/statemachine), and the nav stack itself (internal/node/nav),
// the same loop cmd/track-navigator runs.
//
// Vision is the one subsystem that does not become a target here: ADR 0068
// keeps it in Python behind a sidecar, so this board's side of it is the
// detections subscription the nav loop already opens, not a driver loop of its
// own.
//
// When the hardware profile selects a Pico 2 as the actuation board (the pico2
// profile overlays src/config/hardware/board.toml's kind), the board on
// board.toml's serial_port is served by the picolink target
// (internal/node/picolink, adr:0098-pico-actuation-board-and-portable-cores):
// it answers ackermann_cmd and publishes motor_status, joint_states and
// button_event on the Pico's behalf, exactly as the Zero's loops do for
// themselves. The same profile makes cmd/pi-zero refuse to drive, so the
// subjects never have two publishers. Otherwise actuation stays with the Zero.
package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"path/filepath"
	"syscall"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/capture"
	nodeimu "github.com/teamvoltimor/vtitan/src/go/internal/node/imu"
	nodelidar "github.com/teamvoltimor/vtitan/src/go/internal/node/lidar"
	nodenav "github.com/teamvoltimor/vtitan/src/go/internal/node/nav"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	nodestatemachine "github.com/teamvoltimor/vtitan/src/go/internal/node/statemachine"
	nodetelemetry "github.com/teamvoltimor/vtitan/src/go/internal/node/telemetry"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/camera"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/imu"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/lidar"
	"github.com/teamvoltimor/vtitan/src/go/pkg/supervise"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

type cliConfig struct {
	cmdkit.Common

	fps        float64
	video      bool
	photoEvery time.Duration

	backendAddr string
	robotID     string

	direction string
	challenge string
	navRateHz float64
	record    bool

	picoCommandTimeout time.Duration
	picoMotorInvert    bool
}

// Capture defaults, matching cmd/capture-node's flags so a pi5 run behaves
// the same when neither is overridden.
const (
	defaultFPS           = 15.0
	defaultPhotoInterval = 10 * time.Second
)

// repoRoot walks up from the working directory to the repo root (the directory
// containing other/data), so robot.toml can be located without an absolute
// path.
func repoRoot() (string, error) {
	dir, err := os.Getwd()
	if err != nil {
		return "", fmt.Errorf("pi5: getting working directory: %w", err)
	}
	for {
		if info, statErr := os.Stat(filepath.Join(dir, "other", "data")); statErr == nil && info.IsDir() {
			return dir, nil
		}
		parent := filepath.Dir(dir)
		if parent == dir {
			return "", errors.New("pi5: repo root (other/data) not found walking up from cwd")
		}
		dir = parent
	}
}

func main() {
	os.Exit(runMain())
}

func runMain() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	var cfg cliConfig
	fs := flag.NewFlagSet("pi5", flag.ContinueOnError)
	cfg.RegisterNATSURL(fs)
	fs.StringVar(&cfg.NodeName, "name", "pi5", "NATS client name")
	cfg.RegisterRunsRoot(fs, "runs root dir (default: repo-root other/data/live/runs)")
	cfg.RegisterConfigRoot(fs, "repo root for robot.toml (VTITAN_HARDWARE_PROFILE selects the active profile)")
	cfg.RegisterProfiles(fs)
	fs.Float64Var(&cfg.fps, "fps", defaultFPS, "capture frame rate")
	fs.BoolVar(&cfg.video, "video", true, "record the debug video")
	fs.DurationVar(&cfg.photoEvery, "photo-interval", defaultPhotoInterval, "periodic dataset-photo cadence (0 = off)")
	fs.StringVar(
		&cfg.backendAddr,
		"backend-addr",
		nodestatemachine.DefaultBackendAddr,
		"backend gRPC address (host:port)",
	)
	fs.StringVar(
		&cfg.robotID,
		"robot-id",
		"",
		"robot ID on the backend command channel; empty disables the command channel",
	)
	fs.StringVar(
		&cfg.direction,
		"direction",
		nodenav.DirectionUndetermined,
		"travel direction for the round: cw, ccw, or undetermined (default)",
	)
	fs.StringVar(
		&cfg.challenge,
		"challenge",
		nodenav.ChallengeOpen,
		"which challenge this round runs: open (default) or obstacles",
	)
	fs.Float64Var(&cfg.navRateHz, "nav-rate-hz", nodenav.DefaultRateHz, "navigator Step rate")
	fs.BoolVar(&cfg.record, "record", false, "record the run as an MCAP bag under the runs root")
	fs.DurationVar(
		&cfg.picoCommandTimeout,
		"pico-command-timeout",
		picolink.DefaultCommandTimeout,
		"Pico board: safety-stop the drive if no AckermannCmd arrives within this duration "+
			"(the Zero's --motor-command-timeout)",
	)
	fs.BoolVar(&cfg.picoMotorInvert, "pico-motor-invert", false,
		"Pico board: flip the drive's sign convention (the Zero's --motor-invert)")
	if err := fs.Parse(os.Args[1:]); err != nil {
		return 1
	}

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	camCfg := loadCamera(cfg, logger)

	// Which board actuates is decided once, here: a wrong answer leaves the
	// car with no driver or with two, so an unreadable board.toml stops the
	// process instead of being retried.
	board, err := hwconfig.ActuationBoard(cfg.ConfigRoot)
	if err != nil {
		logger.Error("pi5: resolving the actuation board", "error", err)
		return 1
	}
	logger.Info("pi5: actuation board", "kind", board.Kind, "serial_port", board.SerialPort)

	supervisor, err := supervise.New(supervise.DefaultConfig(), logger)
	if err != nil {
		logger.Error("pi5: creating supervisor", "error", err)
		return 1
	}

	// Each subsystem runs as its own supervised goroutine: a panic or a failure
	// in one is restarted with backoff and never takes the process down with it
	// (pkg/supervise). Each opens its own NATS connection, so a restart
	// reconnects rather than inheriting a half-dead one.
	//
	// Nav is last in the slice deliberately: it is the only target that commands
	// the actuators, so on a clean start the sensors it reads are already
	// publishing by the time it first steps.
	captureCfg := capture.Config{
		Camera:          camCfg,
		NATS:            nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
		RunsRoot:        cfg.RunsRoot,
		FPS:             cfg.fps,
		Video:           cfg.video,
		PhotoInterval:   cfg.photoEvery,
		PhotoSubdir:     "captures",
		PhotoRequireDet: false,
	}
	imuCfg := nodeimu.Config{
		Driver: imu.Config{Port: imu.DefaultPort, BaudRate: imu.DefaultBaudRate},
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
	}
	// The IMU wiring is a hardware fact, so it comes from the active profile
	// rather than from a pi5 flag: --config-root plus VTITAN_HARDWARE_PROFILE
	// select which of the four bno08x_* profiles is live, exactly as
	// cmd/imu-node resolves it. With no config root, the driver defaults apply
	// so a dev machine still starts (the target then fails to open the port and
	// is restarted with backoff, which is the intended behavior, not an error).
	lidarCfg := nodelidar.Config{
		Driver: lidar.Config{Port: lidar.DefaultPort, BaudRate: lidar.DefaultBaudRate},
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
	}
	if cfg.ConfigRoot != "" {
		imuCfg.Driver = hwconfig.IMU(logger, cfg.ConfigRoot)
		lidarCfg.Driver = hwconfig.LIDAR(logger, cfg.ConfigRoot)
	}

	telemetryCfg := nodetelemetry.Config{
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
		RateHz: nodetelemetry.DefaultRateHz,
	}

	navCfg := nodenav.Config{
		NATSURL:    cfg.NATSURL,
		NodeName:   cfg.NodeName,
		ConfigRoot: cfg.ConfigRoot,
		Profiles:   cfg.Profiles,
		RunsRoot:   cfg.RunsRoot,
		Direction:  cfg.direction,
		Challenge:  cfg.challenge,
		RateHz:     cfg.navRateHz,
		Record:     cfg.record,
	}

	targets := []supervise.Target{
		{Name: "capture", Fn: func(ctx context.Context) error {
			return capture.Run(ctx, captureCfg, logger)
		}},
		{Name: "imu", Fn: func(ctx context.Context) error {
			return nodeimu.Run(ctx, imuCfg, logger)
		}},
		{Name: "lidar", Fn: func(ctx context.Context) error {
			return nodelidar.Run(ctx, lidarCfg, logger)
		}},
		{Name: "telemetry", Fn: func(ctx context.Context) error {
			return nodetelemetry.Run(ctx, telemetryCfg, logger)
		}},
	}

	// The Pico link joins only when the profile selects the Pico, and before
	// nav, since it is what carries nav's commands to the actuators. The
	// servo and motor profile is resolved inside the target, so a missing one
	// is retried with backoff like an absent serial port rather than taking
	// the board process down.
	if board.Kind == hardware.HardwareBoardKindPico2 {
		targets = append(targets, supervise.Target{Name: "picolink", Fn: func(ctx context.Context) error {
			sessionCfg, cfgErr := picolink.SessionConfigFor(
				logger, cfg.ConfigRoot, cfg.picoMotorInvert, cfg.picoCommandTimeout,
			)
			if cfgErr != nil {
				return cfgErr //nolint:wrapcheck // already wrapped with "picolink: ..." context
			}
			return picolink.Run(ctx, picolink.Config{
				Port:    board.SerialPort,
				NATS:    nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
				Session: sessionCfg,
			}, logger)
		}})
	}

	targets = append(targets, supervise.Target{Name: "nav", Fn: func(ctx context.Context) error {
		return nodenav.Run(ctx, logger, navCfg)
	}})

	// The backend command channel is optional: it is how an operator starts and
	// stops a round remotely, and a race can run without it. Registering it with
	// no robot ID would leave a target redialling a backend that was never
	// configured, so it joins only when one is given.
	if cfg.robotID != "" {
		smCfg := nodestatemachine.Config{
			NATS:        nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
			BackendAddr: cfg.backendAddr,
			RobotID:     cfg.robotID,
		}
		targets = append(targets, supervise.Target{
			Name: "state-machine",
			Fn: func(ctx context.Context) error {
				return nodestatemachine.Run(ctx, smCfg, logger)
			},
		})
	}

	if err = supervisor.RunAll(ctx, targets...); err != nil {
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
func loadCamera(cfg cliConfig, logger *slog.Logger) camera.Config {
	profiles := profile.ParseNames(cfg.Profiles)
	basePath := cfg.ConfigRoot
	if basePath == "" {
		root, err := repoRoot()
		if err != nil {
			logger.Warn("pi5: repo root not found, defaulting to synthetic camera", "error", err)
			return camera.Config{Source: camera.SourceSynthetic, FPS: cfg.fps}
		}
		basePath = filepath.Join(root, profile.DefaultRobotTOMLPath)
	}
	rc, err := profile.LoadRobotConfig(basePath, profiles)
	if err != nil {
		logger.Warn("pi5: no robot config loaded, defaulting to synthetic camera", "error", err)
		return camera.Config{Source: camera.SourceSynthetic, FPS: cfg.fps}
	}
	cam := rc.Camera
	// robot.toml's [camera] section carries the mount and sensor geometry, not
	// the capture source or device -- those live in the camera driver's own
	// hardware/camera/config.toml. There is no robot.toml key for either, so
	// the source stays synthetic here and the driver config owns the rest.
	return camera.Config{
		Source:      camera.SourceSynthetic,
		Width:       cam.Width,
		Height:      cam.Height,
		FPS:         cfg.fps,
		NATSSubject: "vtitan.sensor.v1.camera",
	}
}
