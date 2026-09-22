// Command track-navigator wires the nav stack (track model, corridor-width
// belief, navigator) to live NATS topics, for both isolated bench testing
// and as the real Pi 5 navigator process (a sibling of lidar-node/imu-node/
// motor-node under systemd, matching Python's track_navigator_node running
// with no --metadata).
//
// It wires the NATS-backed controllers.HardwareGateway (internal/adapters/natsgw)
// to the navigator composition root (internal/nav/navigator) and drives Step at
// the nav control rate. The gateway subscribes to vtitan.sensor.v1.scan and
// vtitan.sensor.v1.imu (published by cmd/lidar-node and the IMU node) plus
// vtitan.actuation.v1.joint_states (the motor node's encoder feedback, which
// supplies the wheel odometry bay exit bounds its legs by), estimates
// pose in-process via localization.LidarLocalizer, and publishes drive commands
// on vtitan.actuation.v1.ackermann_cmd (consumed by the motor node) — no new
// NATS subjects are introduced here.
//
// No --metadata flag exists, matching what Python's track_navigator_node.py
// says competition requires: WRO randomizes the inner walls before each
// round, so a told layout cannot be right on the day. The corridor geometry
// is always estimated from LIDAR (see newBlindLayout, widthbelief.Layout),
// and only the travel direction is optionally told via --direction.
//
// --challenge selects Open (default) or Obstacles. Obstacles wires a
// signrouter.SignRouter built EMPTY (no told sign layout, for the same
// randomization reason) and subscribes to the vision sidecar's detections
// (src/vision/nats_sidecar.py) over NATS via internal/adapters/natsvision, so
// signs are discovered from the camera during the creep exactly as
// newBlindLayout discovers the corridor widths from LIDAR.
package main

import (
	"context"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	nodenav "github.com/teamvoltimor/vtitan/src/go/internal/node/nav"
)

// cliConfig holds every flag track-navigator accepts.
type cliConfig struct {
	cmdkit.Common

	direction string
	challenge string
	rateHz    float64
	record    bool
}

// exit codes: 0 means track-navigator ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable runtime
// error.
const (
	exitOK    = 0
	exitError = 1
)

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "track-navigator",
		Short: "Run the nav stack (navigator + controllers) against live NATS topics",
		Long: "track-navigator wires the NATS-backed controllers.HardwareGateway to the\n" +
			"navigator composition root, subscribes to scan/IMU, estimates pose in-process,\n" +
			"and steps the navigator (publishing AckermannCmd) at --rate-hz.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return runNav(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	cfg.RegisterNATSURL(flags)
	cfg.RegisterNodeName(flags, "track-navigator")
	flags.Float64Var(&cfg.rateHz, "rate-hz", nodenav.DefaultRateHz, "navigator Step rate")
	flags.BoolVar(
		&cfg.record,
		"record",
		false,
		"record the run to other/data/live/runs as a run_<stamp>/ (MCAP bag of /scan + /nav_debug); video/photos are captured separately by cmd/capture-node",
	)
	cfg.RegisterRunsRoot(flags, "runs root dir for --record (default: repo-root other/data/live/runs)")
	cfg.RegisterConfigRoot(flags,
		"repo root to read the shipped TOML tree from; empty runs on Go literal defaults")
	cfg.RegisterProfiles(flags)
	flags.StringVar(&cfg.direction, "direction", nodenav.DirectionUndetermined,
		"travel direction for the round: cw, ccw, or undetermined (default). "+
			"undetermined means the robot creeps and infers it from LIDAR, matching "+
			"what competition requires -- WRO draws the direction on the day.")
	flags.StringVar(&cfg.challenge, "challenge", nodenav.ChallengeOpen,
		"which challenge this round runs: open (default) or obstacles. obstacles "+
			"wires a SignRouter (blind sign discovery from the vision sidecar's "+
			"detections, corridor width fixed at 1.0 m by rule) and switches the "+
			"collision/speed/pursuit tuning to their Obstacles overrides.")

	return cmd
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so deferred cleanup (closing NATS
// subscriptions/connection, releasing the signal.NotifyContext) actually runs
// before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("track-navigator run failed", "error", err)
		return exitError
	}
	return exitOK
}

// runNav translates this binary's flags into the shared nav loop's Config and
// hands off to internal/node/nav -- the same loop cmd/pi5 supervises, so bench
// runs and board runs cannot drift apart.
func runNav(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	//nolint:wrapcheck // nodenav.Run's errors already carry their own package prefix
	return nodenav.Run(ctx, logger, nodenav.Config{
		NATSURL:    cfg.NATSURL,
		NodeName:   cfg.NodeName,
		ConfigRoot: cfg.ConfigRoot,
		Profiles:   cfg.Profiles,
		RunsRoot:   cfg.RunsRoot,
		Direction:  cfg.direction,
		Challenge:  cfg.challenge,
		RateHz:     cfg.rateHz,
		Record:     cfg.record,
	})
}
