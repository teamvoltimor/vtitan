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
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	nodelidar "github.com/teamvoltimor/vtitan/src/go/internal/node/lidar"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/lidar"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// cliConfig holds every flag lidar-node accepts.
type cliConfig struct {
	cmdkit.Common

	port     string
	baudRate int
}

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
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "lidar-node")
	flags.StringVar(&cfg.port, "port", lidar.DefaultPort, "LIDAR serial port")
	flags.IntVar(&cfg.baudRate, "baud-rate", lidar.DefaultBaudRate, "LIDAR serial baud rate")
	cfg.RegisterConfigRoot(flags,
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"overrides --port/--baud-rate when set")

	return cmd
}

// run builds the driver config this binary was asked for and hands it to the
// shared publish loop in internal/node/lidar -- the same loop cmd/pi5
// supervises, so bench runs and board runs cannot drift apart.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	drvCfg := lidar.Config{Port: cfg.port, BaudRate: cfg.baudRate}
	if cfg.ConfigRoot != "" {
		drvCfg = hwconfig.LIDAR(logger, cfg.ConfigRoot)
	}

	//nolint:wrapcheck // nodelidar.Run's errors already carry "lidar: ..."/"nats: ..." context
	return nodelidar.Run(ctx, nodelidar.Config{
		Driver: drvCfg,
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
	}, logger)
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
