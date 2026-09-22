//go:build linux

// Command imu-node is a bench/dev single-subsystem binary for the IMU
// driver, sharing the same internal packages as cmd/pi5 — used for isolated
// hardware bench testing and local debugging without the whole board binary.
//
// It reads BNO08x UART-RVC frames and publishes sensor_msgs/Imu-equivalent
// messages on the `vtitan.sensor.v1.imu` NATS subject.
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
	nodeimu "github.com/teamvoltimor/vtitan/src/go/internal/node/imu"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/imu"
)

// cliConfig holds every flag imu-node accepts.
type cliConfig struct {
	cmdkit.Common

	port     string
	baudRate int
}

// exit codes: 0 means imu-node ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "imu-node",
		Short: "Publish BNO08x UART-RVC readings as sensor_msgs/Imu-equivalent messages over NATS",
		Long: "imu-node reads BNO08x UART-RVC frames from the configured serial port and\n" +
			"publishes them on the vtitan.sensor.v1.imu NATS subject as protobuf Imu\n" +
			"messages, deriving an orientation quaternion from the driver's reported\n" +
			"Euler angles.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "imu-node")
	flags.StringVar(&cfg.port, "port", imu.DefaultPort, "IMU serial port")
	flags.IntVar(&cfg.baudRate, "baud-rate", imu.DefaultBaudRate, "IMU serial baud rate")
	cfg.RegisterConfigRoot(flags,
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"overrides --port/--baud-rate when set")

	return cmd
}

// run builds the driver config this binary was asked for and hands it to the
// shared publish loop in internal/node/imu -- the same loop cmd/pi5 supervises,
// so bench runs and board runs cannot drift apart.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	drvCfg := imu.Config{Port: cfg.port, BaudRate: cfg.baudRate}
	if cfg.ConfigRoot != "" {
		drvCfg = hwconfig.IMU(logger, cfg.ConfigRoot)
	}

	//nolint:wrapcheck // nodeimu.Run's errors already carry "imu: ..."/"nats: ..." context
	return nodeimu.Run(ctx, nodeimu.Config{
		Driver: drvCfg,
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
	}, logger)
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// IMU driver, the NATS connection, releasing the signal.NotifyContext)
// actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("imu-node run failed", "error", err)
		return exitError
	}
	return exitOK
}
