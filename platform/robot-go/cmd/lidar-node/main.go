//go:build linux

// Command lidar-node is a bench/dev single-subsystem binary for the RPLIDAR
// C1 driver, sharing the same internal packages as cmd/pi5 — used for
// isolated hardware bench testing and local debugging without the whole
// board binary.
//
// It reads full 360-degree Scans from the RPLIDAR C1's classic SCAN mode
// and publishes them on the `vtitan.sensor.v1.scan` NATS subject.
package main

import (
	"context"
	"errors"
	"log/slog"
	"os"
	"os/signal"
	"sort"
	"syscall"
	"time"

	"github.com/spf13/cobra"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/lidar"
	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// cliConfig holds every flag lidar-node accepts.
type cliConfig struct {
	natsURL    string
	nodeName   string
	port       string
	baudRate   int
	configRoot string
}

// scanFrameID is this sensor's TF frame, matching
// shared.config.constants.identifiers.TfFrames.LIDAR_LINK.
const scanFrameID = "lidar_link"

// exit codes: 0 means lidar-node ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "lidar-node",
		Short: "Publish RPLIDAR C1 scans as sensor_msgs/LaserScan-equivalent messages over NATS",
		Long: "lidar-node reads full 360-degree Scans from the RPLIDAR C1 over its classic\n" +
			"SCAN serial protocol and publishes them on the vtitan.sensor.v1.scan NATS\n" +
			"subject as protobuf Scan messages.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultDevURL, "nats-server URL")
	flags.StringVar(
		&cfg.nodeName,
		"name",
		"lidar-node",
		"NATS client name, visible in nats-server's connz output",
	)
	flags.StringVar(&cfg.port, "port", lidar.DefaultPort, "LIDAR serial port")
	flags.IntVar(&cfg.baudRate, "baud-rate", lidar.DefaultBaudRate, "LIDAR serial baud rate")
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"overrides --port/--baud-rate when set")

	return cmd
}

// scanMessageFor builds the Scan message to publish for one assembled
// 360-degree Scan.
//
// The classic SCAN protocol streams samples in acquisition order at
// whatever angular spacing the motor's rotation happened to produce --
// unlike EXPRESS_SCAN/ULTRA modes (not implemented, see the lidar package's
// doc.go), it makes no fixed-grid guarantee. sensor_msgs/LaserScan (and
// this repo's scan.proto, which mirrors it) models a regularly-spaced
// angle_min..angle_max sweep with one angle_increment step, so this sorts
// samples by angle and reports their *average* spacing as angle_increment
// rather than inventing a resampling/binning step -- there is no in-repo or
// vendor reference for how such a step should behave (see lidar/doc.go),
// so approximating the real, unevenly-spaced data as-is is preferred over
// guessing at one.
func scanMessageFor(scan lidar.Scan, sinceLastScan time.Duration) *sensorv1.Scan {
	points := append(lidar.Scan(nil), scan...)
	sort.Slice(points, func(i, j int) bool { return points[i].AngleRad < points[j].AngleRad })

	ranges := make([]float32, len(points))
	intensities := make([]float32, len(points))
	for i, pt := range points {
		ranges[i] = float32(pt.RangeM)
		intensities[i] = float32(pt.Quality)
	}

	var angleMin, angleMax, angleIncrement, timeIncrement float32
	if len(points) > 0 {
		angleMin = float32(points[0].AngleRad)
		angleMax = float32(points[len(points)-1].AngleRad)
	}
	if len(points) > 1 {
		angleIncrement = (angleMax - angleMin) / float32(len(points)-1)
		timeIncrement = float32(sinceLastScan.Seconds()) / float32(len(points)-1)
	}

	return &sensorv1.Scan{
		Stamp:   timestamppb.Now(),
		FrameId: scanFrameID,

		AngleMin:       angleMin,
		AngleMax:       angleMax,
		AngleIncrement: angleIncrement,
		TimeIncrement:  timeIncrement,
		ScanTime:       float32(sinceLastScan.Seconds()),
		RangeMin:       lidar.MinRangeM,
		RangeMax:       lidar.MaxRangeM,

		Ranges:      ranges,
		Intensities: intensities,
	}
}

// run wires the LIDAR driver to NATS and blocks until ctx is done or a
// non-cancellation error occurs.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	drvCfg := lidar.Config{Port: cfg.port, BaudRate: cfg.baudRate}
	if cfg.configRoot != "" {
		drvCfg = lidar.ConfigFor(logger, cfg.configRoot)
	}

	drv, err := lidar.New(drvCfg)
	if err != nil {
		return err //nolint:wrapcheck // lidar.New already wraps with "lidar: ..." context
	}
	if err = drv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "lidar: ..." context
	}
	defer func() {
		if closeErr := drv.Close(); closeErr != nil {
			logger.Error("lidar-node: closing LIDAR driver", "error", closeErr)
		}
	}()

	conn, err := nats.Connect(nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	pub := nats.NewPublisher[*sensorv1.Scan](conn, sensorv1.ScanSubject)

	logger.Info("lidar-node: connected", "nats_url", cfg.natsURL, "port", drvCfg.Port)
	return publishLoop(ctx, logger, drv, pub)
}

// publishLoop reads successive Scans from drv and publishes each as a Scan
// message, until ctx is done or Read returns a non-cancellation error.
func publishLoop(
	ctx context.Context,
	logger *slog.Logger,
	drv *lidar.SerialDriver,
	pub *nats.Publisher[*sensorv1.Scan],
) error {
	lastScanAt := time.Now()

	for {
		scan, err := drv.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "lidar: ..." context
		}

		now := time.Now()
		if pubErr := pub.Publish(scanMessageFor(scan, now.Sub(lastScanAt))); pubErr != nil {
			logger.Error("lidar-node: publishing Scan", "error", pubErr)
		}
		lastScanAt = now
	}
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// LIDAR driver, the NATS connection, releasing the signal.NotifyContext)
// actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("lidar-node run failed", "error", err)
		return exitError
	}
	return exitOK
}
