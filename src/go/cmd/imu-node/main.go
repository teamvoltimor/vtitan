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
	"errors"
	"log/slog"
	"os"
	"os/signal"
	"syscall"

	"github.com/spf13/cobra"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/imu"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// cliConfig holds every flag imu-node accepts.
type cliConfig struct {
	natsURL    string
	nodeName   string
	port       string
	baudRate   int
	configRoot string
}

// imuFrameID is this sensor's TF frame, matching
// shared.config.constants.identifiers.TfFrames.IMU_LINK (the value the
// existing ROS2 uart_rvc_node.py publishes Imu messages under).
const imuFrameID = "imu_link"

// exit codes: 0 means imu-node ran and shut down cleanly (including via
// SIGINT/SIGTERM). 1 means it could not start or hit an unrecoverable
// runtime error.
const (
	exitOK    = 0
	exitError = 1
)

// orientationCovarianceUnknown/angularVelocityCovarianceUnknown are the
// row-major 3x3 "no estimate" covariance matrices sensor_msgs/Imu's own
// convention defines (element [0] = -1, the rest 0) -- matches
// uart_rvc_node.py exactly: RVC mode reports Euler angles (used to derive
// orientation) but never angular velocity, so only orientation gets a real
// (if unknown-magnitude) covariance marker and angular velocity is zeroed
// out entirely.
var orientationCovarianceUnknown = [9]float64{-1, 0, 0, 0, 0, 0, 0, 0, 0}

var angularVelocityCovarianceUnknown = [9]float64{-1, 0, 0, 0, 0, 0, 0, 0, 0}

// linearAccelerationCovarianceDiag01 is uart_rvc_node.py's approximate
// diagonal 0.01 covariance for linear acceleration -- not a measured
// sensor spec, the Python driver's own comment calls it approximate too.
var linearAccelerationCovarianceDiag01 = [9]float64{0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01}

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
	flags.StringVar(&cfg.natsURL, "nats-url", nats.DefaultURL(), "nats-server URL")
	flags.StringVar(
		&cfg.nodeName,
		"name",
		"imu-node",
		"NATS client name, visible in nats-server's connz output",
	)
	flags.StringVar(&cfg.port, "port", imu.DefaultPort, "IMU serial port")
	flags.IntVar(&cfg.baudRate, "baud-rate", imu.DefaultBaudRate, "IMU serial baud rate")
	flags.StringVar(&cfg.configRoot, "config-root", "",
		"repo root to load the hardware profile (VTITAN_HARDWARE_PROFILE) from; "+
			"overrides --port/--baud-rate when set")

	return cmd
}

// vec3 builds a sensor_msgs/Vector3-equivalent message.
func vec3(x, y, z float64) *sensorv1.Vector3 {
	return &sensorv1.Vector3{X: x, Y: y, Z: z}
}

// imuMessageFor builds the Imu message to publish for one decoded RVC
// reading.
func imuMessageFor(reading imu.Reading) *sensorv1.Imu {
	q := imu.QuaternionFromEuler(reading.Yaw, reading.Pitch, reading.Roll)

	return &sensorv1.Imu{
		Stamp:   timestamppb.Now(),
		FrameId: imuFrameID,

		Orientation:           &sensorv1.Quaternion{X: q.X, Y: q.Y, Z: q.Z, W: q.W},
		OrientationCovariance: orientationCovarianceUnknown[:],

		AngularVelocity:           vec3(0, 0, 0),
		AngularVelocityCovariance: angularVelocityCovarianceUnknown[:],

		LinearAcceleration:           vec3(reading.XAccel, reading.YAccel, reading.ZAccel),
		LinearAccelerationCovariance: linearAccelerationCovarianceDiag01[:],
	}
}

// run wires the IMU driver to NATS and blocks until ctx is done or a
// non-cancellation error occurs.
func run(ctx context.Context, logger *slog.Logger, cfg cliConfig) error {
	drvCfg := imu.Config{Port: cfg.port, BaudRate: cfg.baudRate}
	if cfg.configRoot != "" {
		drvCfg = imu.ConfigFor(logger, cfg.configRoot)
	}

	drv, err := imu.New(drvCfg)
	if err != nil {
		return err //nolint:wrapcheck // imu.New already wraps with "imu: ..." context
	}
	if err = drv.Connect(ctx); err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "imu: ..." context
	}
	defer func() {
		if closeErr := drv.Close(); closeErr != nil {
			logger.Error("imu-node: closing IMU driver", "error", closeErr)
		}
	}()

	conn, err := nats.Connect(ctx, nats.DefaultConfig(cfg.natsURL, cfg.nodeName))
	if err != nil {
		return err //nolint:wrapcheck // Connect already wraps with "nats: ..." context
	}
	defer conn.Close()

	pub := nats.NewPublisher[*sensorv1.Imu](conn, sensorv1.ImuSubject)

	logger.Info("imu-node: connected", "nats_url", cfg.natsURL, "port", drvCfg.Port)
	return publishLoop(ctx, logger, drv, pub)
}

// publishLoop reads successive Readings from drv and publishes each as an
// Imu message, until ctx is done or Read returns a non-cancellation error.
func publishLoop(
	ctx context.Context,
	logger *slog.Logger,
	drv *imu.RVCDriver,
	pub *nats.Publisher[*sensorv1.Imu],
) error {
	for {
		reading, err := drv.Read(ctx)
		if err != nil {
			if errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return err //nolint:wrapcheck // Read already wraps with "imu: ..." context
		}

		if pubErr := pub.Publish(imuMessageFor(reading)); pubErr != nil {
			logger.Error("imu-node: publishing Imu", "error", pubErr)
		}
	}
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
