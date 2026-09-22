//go:build linux

// Command telemetry-node is a bench/dev single-subsystem binary for the
// telemetry-summary aggregation loop, sharing the same internal packages as
// cmd/pi5 — used for isolated bench testing and local debugging without the
// whole board binary.
//
// It subscribes to the IMU and LIDAR topics other Go nodes publish
// (vtitan.sensor.v1.imu, vtitan.sensor.v1.scan), feeds them into the
// existing internal/telemetry/diag.Aggregator, and publishes the resulting
// TelemetrySummary on vtitan.ui.v1.telemetry_summary for cmd/pi-zero's OLED
// to render. There is no vision detection wire schema yet (see
// diag.Detection's doc comment), so best-detection fields are always
// omitted here, matching Aggregator's own documented behavior for an input
// that has never arrived.
package main

import (
	"context"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	nodetelemetry "github.com/teamvoltimor/vtitan/src/go/internal/node/telemetry"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// cliConfig holds every flag telemetry-node accepts.
type cliConfig struct {
	cmdkit.Common

	rateHz float64
}

// exit codes: 0 means telemetry-node ran and shut down cleanly (including
// via SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "telemetry-node",
		Short: "Aggregate IMU/LIDAR topics into a TelemetrySummary and publish it over NATS",
		Long: "telemetry-node subscribes to vtitan.sensor.v1.imu and vtitan.sensor.v1.scan,\n" +
			"feeds them into the existing telemetry-summary aggregator, and publishes the\n" +
			"result on vtitan.ui.v1.telemetry_summary at --rate-hz for cmd/pi-zero's OLED.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "telemetry-node")
	flags.Float64Var(&cfg.rateHz, "rate-hz", nodetelemetry.DefaultRateHz, "TelemetrySummary publish rate")
	cfg.RegisterConfigRoot(flags,
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"empty uses a zero LIDAR yaw offset")

	return cmd
}

// run hands this binary's flags to the shared aggregation loop in
// internal/node/telemetry -- the same loop cmd/pi5 supervises, so bench runs
// and board runs cannot drift apart.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	//nolint:wrapcheck // nodetelemetry.Run's errors already carry "nats: ..." context
	return nodetelemetry.Run(ctx, nodetelemetry.Config{
		NATS:   nats.DefaultConfig(cfg.NATSURL, cfg.NodeName),
		RateHz: cfg.rateHz,
	}, logger)
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing the
// NATS subscriptions/connection, releasing the signal.NotifyContext)
// actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("telemetry-node run failed", "error", err)
		return exitError
	}
	return exitOK
}
