//go:build linux

// Command pi-zero is the production combined board binary for the Pi Zero:
// motor + button + OLED, run as supervised goroutines in a single process.
// See platform/robot/docs/internal/plans/go-migration-plan.md ("Process
// model").
//
// The motor control loop is internal/node/motor, shared with cmd/motor-node
// rather than duplicated -- see that package's doc.go.
package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/button"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/display/ssd1306"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/encoder"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/motor"
	nodebutton "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/button"
	nodemotor "github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/motor"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/supervise"

	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
	uiv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// cliConfig holds every flag pi-zero accepts.
type cliConfig struct {
	natsURL  string
	nodeName string

	motorCommandTimeout time.Duration
	motorInvert         bool
	configRoot          string

	buttonLine   int
	buttonPullUp bool

	oledI2CBus     int
	oledI2CAddress uint16
	oledWidth      int
	oledHeight     int
}

// defaultButtonLine/defaultButtonPullUp match
// platform/config/hardware/button/gpio.toml's button_gpio_pin/
// pull_up defaults (GPIO4, wired GND-to-pin so a press pulls the line LOW).
const (
	defaultButtonLine   = 4
	defaultButtonPullUp = true
)

// exit codes: 0 means pi-zero ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

// oledLine{One,Two,Three}Y are the top-left y coordinates DrawString draws
// each status line at, spaced one pixel past ssd1306's fixed 5px glyph
// height so consecutive lines don't touch.
const (
	oledLineOneY   = 0
	oledLineTwoY   = 8
	oledLineThreeY = 16
	oledLineX      = 0
)

// oledPercentScale converts a confidence fraction into the whole-percent
// integer renderSummary draws -- the glyph set DrawString supports (digits,
// uppercase letters, space, ':', '-', '.', '%' -- see text.go's glyphs
// table) has no lowercase and no useful sub-percent precision for a status
// glyph readout.
const oledPercentScale = 100

func newRootCmd(cfg *cliConfig, logger *slog.Logger) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "pi-zero",
		Short: "Run the Pi Zero's motor, button, and OLED drivers as supervised NATS-wired goroutines",
		Long: "pi-zero drives the BTS7960 motor from AckermannCmd, publishes button\n" +
			"presses, and renders TelemetrySummary to the OLED -- all three run as\n" +
			"supervised goroutines (internal/supervise) inside one process, sharing a\n" +
			"single NATS connection.",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg)
		},
	}

	flags := cmd.Flags()
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	flags.StringVar(
		&cfg.nodeName,
		"name",
		"pi-zero",
		"NATS client name, visible in nats-server's connz output",
	)

	flags.DurationVar(
		&cfg.motorCommandTimeout,
		"motor-command-timeout",
		nodemotor.DefaultCommandTimeout,
		"safety-stop the drive if no AckermannCmd arrives within this duration",
	)
	flags.BoolVar(&cfg.motorInvert, "motor-invert", false,
		"flip SetSpeed's sign convention, matching motors.toml's drive.reversed")
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"empty uses nodemotor.DefaultSpeedScalePercentPerMPS")

	flags.IntVar(&cfg.buttonLine, "button-line", defaultButtonLine, "button GPIO line offset")
	flags.BoolVar(&cfg.buttonPullUp, "button-pull-up", defaultButtonPullUp,
		"button wired GND-to-pin (pull-up, press = LOW) rather than 3V3-to-pin")

	flags.IntVar(&cfg.oledI2CBus, "oled-i2c-bus", ssd1306.DefaultI2CBus, "OLED I2C bus number")
	flags.Uint16Var(
		&cfg.oledI2CAddress,
		"oled-i2c-address",
		ssd1306.DefaultI2CAddress,
		"OLED I2C address",
	)
	flags.IntVar(&cfg.oledWidth, "oled-width", ssd1306.DefaultWidth, "OLED panel width in pixels")
	flags.IntVar(
		&cfg.oledHeight,
		"oled-height",
		ssd1306.DefaultHeight,
		"OLED panel height in pixels",
	)

	return cmd
}

// buttonLoop publishes every debounced button event drv produces, until ctx
// is done or Read returns a non-cancellation error.
func buttonLoop(
	ctx context.Context,
	logger *slog.Logger,
	drv *button.Driver,
	pub *nats.Publisher[*uiv1.ButtonEvent],
) error {
	for {
		event, err := drv.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "button: ..." context
		}
		if pubErr := pub.Publish(nodebutton.EventMessageFor(event)); pubErr != nil {
			logger.Error("pi-zero: publishing ButtonEvent", "error", pubErr)
		}
	}
}

// renderSummary draws summary's fields into a fresh Framebuffer sized to
// cfg, matching oled_display_node.py's status-line content (LIDAR
// clearances, yaw, best detection) but not its PIL-based rendering or
// multi-page layout -- see internal/driver/display/ssd1306/doc.go's scope
// note: page-orchestration is deliberately out of scope for this driver
// port, so this renders one fixed status screen, not oled_display_node.py's
// full page set.
func renderSummary(
	cfg ssd1306.Config,
	summary *uiv1.TelemetrySummary,
) (*ssd1306.Framebuffer, error) {
	fb, err := ssd1306.NewFramebuffer(cfg.Width, cfg.Height)
	if err != nil {
		return nil, err //nolint:wrapcheck // NewFramebuffer already wraps with "ssd1306: ..." context
	}

	ssd1306.DrawString(fb, oledLineX, oledLineOneY, fmt.Sprintf(
		"F%d L%d R%d",
		int(
			summary.GetLidarFrontCm(),
		),
		int(summary.GetLidarLeftCm()),
		int(summary.GetLidarRightCm()),
	), true)
	ssd1306.DrawString(
		fb,
		oledLineX,
		oledLineTwoY,
		fmt.Sprintf("YAW%d", int(summary.GetGyroYawDeg())),
		true,
	)

	if summary.GetHasBestDetection() {
		ssd1306.DrawString(fb, oledLineX, oledLineThreeY, fmt.Sprintf(
			"%s %d%%",
			strings.ToUpper(summary.GetBestDetectionClassId()),
			int(summary.GetBestDetectionConfidence()*oledPercentScale),
		), true)
	}

	return fb, nil
}

// oledLoop renders and writes every TelemetrySummary received on sub, until
// ctx is done or Read returns a non-cancellation error.
func oledLoop(
	ctx context.Context,
	logger *slog.Logger,
	cfg ssd1306.Config,
	drv *ssd1306.Driver,
	sub *nats.Subscriber[*uiv1.TelemetrySummary],
) error {
	for {
		summary, err := sub.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "nats: ..." context
		}

		fb, renderErr := renderSummary(cfg, summary)
		if renderErr != nil {
			logger.Error("pi-zero: rendering TelemetrySummary", "error", renderErr)
			continue
		}
		if writeErr := drv.WriteFramebuffer(ctx, fb); writeErr != nil {
			logger.Error("pi-zero: writing OLED framebuffer", "error", writeErr)
		}
	}
}

// run connects every driver and NATS subscription/publisher, then runs the
// motor/button/OLED loops as supervised goroutines until ctx is done.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	motorCfg := motor.DefaultConfig()
	buttonCfg := button.Config{
		GPIOChip:     button.DefaultGPIOChip,
		Line:         cfg.buttonLine,
		PullUp:       cfg.buttonPullUp,
		PollInterval: button.DefaultPollInterval,
		Thresholds:   button.DefaultThresholds(),
	}
	oledCfg := ssd1306.Config{
		Width: cfg.oledWidth, Height: cfg.oledHeight,
		I2CAddress: cfg.oledI2CAddress, I2CBus: cfg.oledI2CBus,
	}
	if cfg.configRoot != "" {
		motorCfg = motor.ConfigFor(logger, cfg.configRoot)
		buttonCfg = button.ConfigFor(logger, cfg.configRoot)
		oledCfg = ssd1306.ConfigFor(logger, cfg.configRoot)
	}
	motorCfg.Invert = cfg.motorInvert

	motorDrv, err := motor.New(motorCfg)
	if err != nil {
		return err //nolint:wrapcheck // motor.New already wraps with "motor: ..." context
	}
	if err = motorDrv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "motor: ..." context
	}
	defer closeLogged(logger, "motor driver", motorDrv.Close)

	buttonDrv, err := button.New(buttonCfg)
	if err != nil {
		return err //nolint:wrapcheck // button.New already wraps with "button: ..." context
	}
	if err = buttonDrv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "button: ..." context
	}
	defer closeLogged(logger, "button driver", buttonDrv.Close)

	oledDrv, err := ssd1306.New(oledCfg)
	if err != nil {
		return err //nolint:wrapcheck // ssd1306.New already wraps with "ssd1306: ..." context
	}
	if err = oledDrv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "ssd1306: ..." context
	}
	defer closeLogged(logger, "OLED driver", oledDrv.Close)

	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	ackermannSub, err := nats.NewSubscriber(
		conn,
		actuationv1.AckermannCmdSubject,
		func() *actuationv1.AckermannCmd {
			return &actuationv1.AckermannCmd{}
		},
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer closeLogged(logger, "AckermannCmd subscription", ackermannSub.Close)

	summarySub, err := nats.NewSubscriber(
		conn,
		uiv1.TelemetrySummarySubject,
		func() *uiv1.TelemetrySummary {
			return &uiv1.TelemetrySummary{}
		},
	)
	if err != nil {
		return err //nolint:wrapcheck // NewSubscriber already wraps with "nats: ..." context
	}
	defer closeLogged(logger, "TelemetrySummary subscription", summarySub.Close)

	motorStatusPub := nats.NewPublisher[*actuationv1.MotorStatus](
		conn,
		actuationv1.MotorStatusSubject,
	)
	buttonEventPub := nats.NewPublisher[*uiv1.ButtonEvent](conn, uiv1.ButtonEventSubject)

	supervisor, err := supervise.New(supervise.DefaultConfig(), logger)
	if err != nil {
		return err //nolint:wrapcheck // New already wraps with "supervise: ..." context
	}

	mLoop := nodemotor.NewLoop(
		logger,
		motorDrv,
		motorStatusPub,
		nodemotor.SpeedScaleFor(logger, cfg.configRoot),
	)

	// The encoder is optional: a Zero with no encoder wired (or no motor
	// profile active, so counts_per_rev is unknown) still drives, it just
	// publishes no odometry -- the ok=false state natsgw.Gateway models.
	// Fabricating a counts_per_rev to keep the loop alive would produce
	// confident, wrong distances instead.
	var feedbackTargets []supervise.Target
	encCfg, encCfgErr := encoder.ConfigFor(cfg.configRoot)
	switch {
	case encCfgErr != nil:
		logger.Warn("pi-zero: no wheel encoder configured, not publishing joint_states",
			"error", encCfgErr)
	default:
		enc, encErr := encoder.New(encCfg)
		if encErr != nil {
			return fmt.Errorf("pi-zero: %w", encErr)
		}
		if encErr = enc.Connect(ctx); encErr != nil {
			return fmt.Errorf("pi-zero: %w", encErr)
		}
		defer closeLogged(logger, "wheel encoder", enc.Close)

		feedback := nodemotor.NewFeedback(
			logger,
			enc,
			nats.NewPublisher[*actuationv1.JointStates](conn, actuationv1.JointStatesSubject),
			nodemotor.DefaultFeedbackInterval,
		)
		feedbackTargets = append(feedbackTargets, supervise.Target{
			Name: "wheel-odometry",
			Fn:   feedback.Run,
		})
		logger.Info("pi-zero: publishing wheel odometry",
			"subject", actuationv1.JointStatesSubject,
			"pin_a", encCfg.PinA, "pin_b", encCfg.PinB,
			"counts_per_rev", encCfg.CountsPerRev)
	}

	logger.Info("pi-zero: connected", "nats_url", cfg.natsURL)
	targets := []supervise.Target{
		{Name: "motor", Fn: func(ctx context.Context) error {
			return mLoop.Run(ctx, ackermannSub, cfg.motorCommandTimeout)
		}},
		{Name: "button", Fn: func(ctx context.Context) error {
			return buttonLoop(ctx, logger, buttonDrv, buttonEventPub)
		}},
		{Name: "oled", Fn: func(ctx context.Context) error {
			return oledLoop(ctx, logger, oledCfg, oledDrv, summarySub)
		}},
	}
	targets = append(targets, feedbackTargets...)
	if err = supervisor.RunAll(ctx, targets...); err != nil {
		return fmt.Errorf("pi-zero: %w", err)
	}
	return nil
}

// closeLogged calls closeFn and logs a failure under label rather than
// returning it -- used from defer, where every driver/subscription in run
// must still attempt its own teardown even if an earlier one failed.
func closeLogged(logger *slog.Logger, label string, closeFn func() error) {
	if err := closeFn(); err != nil {
		logger.Error("pi-zero: closing resource", "label", label, "error", err)
	}
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer` cleanup (closing
// every driver, the NATS connection/subscriptions, releasing the
// signal.NotifyContext) actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("pi-zero run failed", "error", err)
		return exitError
	}
	return exitOK
}
