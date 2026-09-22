package smoke_test

import (
	"context"
	"errors"
	"log/slog"
	"math"
	"sync/atomic"
	"testing"
	"time"

	natssrv "github.com/nats-io/nats-server/v2/server"
	natstest "github.com/nats-io/nats-server/v2/test"
	"google.golang.org/protobuf/types/known/timestamppb"

	nodenav "github.com/teamvoltimor/vtitan/src/go/internal/node/nav"
	nodetelemetry "github.com/teamvoltimor/vtitan/src/go/internal/node/telemetry"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/supervise"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

const (
	serverReadyTimeout = 5 * time.Second
	serverPollInterval = 10 * time.Millisecond

	// scanRays matches the RPLIDAR C1's sweep resolution closely enough that
	// the navigator's corridor classification sees a realistic ray count.
	scanRays = 360
	// corridorHalfWidthM puts walls where an Open Challenge corridor has
	// them, so the synthesized sweep is a corridor rather than a circle.
	corridorHalfWidthM = 0.5
	// publishRateHz feeds scans faster than the nav loop consumes them, so
	// the test does not depend on the two staying in lockstep.
	publishRateHz = 50.0
	// settleTimeout bounds how long the composition may take to turn its
	// first scan into a drive command.
	settleTimeout = 15 * time.Second
	// restartTimeout covers one backoff.DefaultInitial (1s) plus slack, which
	// is how long the supervisor waits before retrying a failed target.
	restartTimeout = 5 * time.Second
	// pollInterval is how often the restart wait re-checks.
	pollInterval = 20 * time.Millisecond
	// minLateralCm/maxLateralCm bracket the side clearance the aggregator
	// should report for the synthesized corridor -- wide enough to survive a
	// change in how the median is taken (it measured 52.5 cm for a 50 cm
	// wall), narrow enough to exclude the 8 m open space an unfed or
	// misrouted scan would leave.
	minLateralCm = 30.0
	maxLateralCm = 80.0
)

// waitForCommand reads drive commands until one satisfies want, or ctx ends.
func waitForCommand(
	ctx context.Context,
	t *testing.T,
	sub *nats.Subscriber[*actuationv1.AckermannCmd],
	want func(*actuationv1.AckermannCmd) bool,
) bool {
	t.Helper()

	for {
		cmd, err := sub.Read(ctx)
		if err != nil {
			return false
		}
		if want(cmd) {
			return true
		}
	}
}

// waitFor polls cond until it holds or timeout elapses, reporting whether it
// held. Used instead of a bare sleep so a fast machine does not pay for a
// slow one's worst case.
func waitFor(t *testing.T, timeout time.Duration, cond func() bool) bool {
	t.Helper()

	deadline := time.Now().Add(timeout)
	for time.Now().Before(deadline) {
		if cond() {
			return true
		}
		time.Sleep(pollInterval)
	}
	return cond()
}

// startTestServer starts an in-process, ephemeral-port nats-server for the
// duration of the test -- nats-server is a Go library, so no external process
// or container is involved.
func startTestServer(t *testing.T) string {
	t.Helper()

	opts := natstest.DefaultTestOptions
	opts.Port = -1
	srv := natstest.RunServer(&opts)
	t.Cleanup(func() {
		srv.Shutdown()
		srv.WaitForShutdown()
	})
	waitForServerReady(t, srv)
	return srv.ClientURL()
}

func waitForServerReady(t *testing.T, srv *natssrv.Server) {
	t.Helper()

	deadline := time.Now().Add(serverReadyTimeout)
	for !srv.ReadyForConnections(serverPollInterval) {
		if time.Now().After(deadline) {
			t.Fatal("nats-server did not become ready in time")
		}
	}
}

// corridorScan synthesizes one sweep down a straight corridor: rays to the
// sides terminate on a wall at corridorHalfWidthM, rays ahead and behind run
// long. It is deliberately simple -- the assertion is that a scan produces a
// command, not that the command is any particular value.
func corridorScan() *sensorv1.Scan {
	ranges := make([]float32, scanRays)
	for i := range ranges {
		angle := (2 * math.Pi * float64(i) / float64(scanRays)) - math.Pi
		lateral := math.Abs(math.Sin(angle))
		if lateral < 1e-3 {
			ranges[i] = 8.0
			continue
		}
		ranges[i] = float32(math.Min(8.0, corridorHalfWidthM/lateral))
	}
	return &sensorv1.Scan{
		Stamp:          timestamppb.Now(),
		FrameId:        "lidar_link",
		AngleMin:       float32(-math.Pi),
		AngleMax:       float32(math.Pi),
		AngleIncrement: float32(2 * math.Pi / float64(scanRays)),
		RangeMin:       0.05,
		RangeMax:       12.0,
		Ranges:         ranges,
	}
}

// levelIMU is a robot sitting flat and facing along its start axis.
func levelIMU() *sensorv1.Imu {
	return &sensorv1.Imu{
		Stamp:                 timestamppb.Now(),
		FrameId:               "imu_link",
		Orientation:           &sensorv1.Quaternion{W: 1},
		OrientationCovariance: []float64{-1, 0, 0, 0, 0, 0, 0, 0, 0},
		AngularVelocity:       &sensorv1.Vector3{},
		LinearAcceleration:    &sensorv1.Vector3{},
	}
}

// TestBoardComposition_ScanInDriveCommandOut is the seam test: the nav and
// telemetry run loops are supervised together exactly as cmd/pi5 supervises
// them, a scan is published on the real subject, and a drive command is
// expected on the real subject in response.
//
// A third target fails on every attempt, standing in for the serial drivers
// on a machine with no hardware. Asserting that the other two keep working
// through its restarts is the point: one subsystem crash-looping must never
// take the board process down, which is the property supervise exists for and
// the one a composition change is most likely to break.
func TestBoardComposition_ScanInDriveCommandOut(t *testing.T) {
	t.Parallel()

	url := startTestServer(t)
	logger := slog.New(slog.DiscardHandler)

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()

	var brokenAttempts atomic.Int64
	supervisor, err := supervise.New(supervise.DefaultConfig(), logger)
	if err != nil {
		t.Fatalf("supervise.New: %v", err)
	}

	navCfg := nodenav.Config{
		NATSURL:   url,
		NodeName:  "smoke-nav",
		Direction: nodenav.DirectionUndetermined,
		Challenge: nodenav.ChallengeOpen,
		RateHz:    nodenav.DefaultRateHz,
	}
	telemetryCfg := nodetelemetry.Config{
		NATS:   nats.DefaultConfig(url, "smoke-telemetry"),
		RateHz: nodetelemetry.DefaultRateHz,
	}

	done := make(chan error, 1)
	go func() {
		done <- supervisor.RunAll(ctx,
			supervise.Target{Name: "nav", Fn: func(ctx context.Context) error {
				return nodenav.Run(ctx, logger, navCfg)
			}},
			supervise.Target{Name: "telemetry", Fn: func(ctx context.Context) error {
				return nodetelemetry.Run(ctx, telemetryCfg, logger)
			}},
			supervise.Target{Name: "broken-driver", Fn: func(context.Context) error {
				brokenAttempts.Add(1)
				return errors.New("smoke: no hardware here, as on a dev machine")
			}},
		)
	}()

	conn, err := nats.Connect(ctx, nats.DefaultConfig(url, "smoke-probe"))
	if err != nil {
		t.Fatalf("nats.Connect: %v", err)
	}
	defer conn.Close()

	cmdSub, err := nats.NewSubscriber[actuationv1.AckermannCmd](conn, actuationv1.AckermannCmdSubject)
	if err != nil {
		t.Fatalf("subscribing to drive commands: %v", err)
	}
	defer func() { _ = cmdSub.Close() }()

	summarySub, err := nats.NewSubscriber[uiv1.TelemetrySummary](conn, uiv1.TelemetrySummarySubject)
	if err != nil {
		t.Fatalf("subscribing to telemetry: %v", err)
	}
	defer func() { _ = summarySub.Close() }()

	scanPub := nats.NewPublisher[*sensorv1.Scan](conn, sensorv1.ScanSubject)
	imuPub := nats.NewPublisher[*sensorv1.Imu](conn, sensorv1.ImuSubject)

	feedCtx, stopFeed := context.WithCancel(ctx)
	defer stopFeed()
	go func() {
		ticker := time.NewTicker(time.Duration(float64(time.Second) / publishRateHz))
		defer ticker.Stop()
		for {
			select {
			case <-feedCtx.Done():
				return
			case <-ticker.C:
				_ = scanPub.Publish(corridorScan())
				_ = imuPub.Publish(levelIMU())
			}
		}
	}()

	readCtx, cancelRead := context.WithTimeout(ctx, settleTimeout)
	defer cancelRead()

	// A command ARRIVING proves only that the navigator is alive: it steps and
	// publishes every tick whether or not a scan ever reached it, and with no
	// scan it commands a standstill. So the assertion is on the speed, which
	// is the value that separates "wired" from "running": measured 0.133 m/s
	// with these scans against exactly 0 without them.
	if !waitForCommand(readCtx, t, cmdSub, func(cmd *actuationv1.AckermannCmd) bool {
		return cmd.GetSpeed() > 0
	}) {
		t.Fatalf("no drive command with speed > 0 within %s: scans are not reaching the controller",
			settleTimeout)
	}

	// Same reasoning for telemetry, and here the value can be checked against
	// the geometry that was synthesized: the corridor walls are at
	// corridorHalfWidthM, and an unfed aggregator reports 0 for every field.
	summary, err := summarySub.Read(readCtx)
	if err != nil {
		t.Fatalf("no telemetry summary within %s: %v", settleTimeout, err)
	}
	if summary.GetLidarFrontCm() <= 0 {
		t.Errorf("summary front clearance = %v cm, want > 0 (the aggregator saw no scan)",
			summary.GetLidarFrontCm())
	}
	lateralCm := summary.GetLidarLeftCm()
	if lateralCm < minLateralCm || lateralCm > maxLateralCm {
		t.Errorf("summary left clearance = %v cm, want within [%v, %v]: it should reflect the %v m "+
			"corridor wall that was synthesized, not open space",
			lateralCm, minLateralCm, maxLateralCm, corridorHalfWidthM)
	}

	// The first retry lands one backoff.DefaultInitial after the first
	// failure, which is longer than the data path takes to produce its first
	// command -- so this waits rather than reading the counter immediately.
	if !waitFor(t, restartTimeout, func() bool { return brokenAttempts.Load() >= 2 }) {
		t.Errorf("broken target ran %d times in %s, want >= 2 (it must be restarted, not abandoned)",
			brokenAttempts.Load(), restartTimeout)
	}

	// Everything above happened while the broken target was crash-looping, so
	// reaching here already proves it did not take the process down. Shutting
	// down cleanly is the other half of the contract.
	stopFeed()
	cancel()
	select {
	case runErr := <-done:
		if runErr != nil && !errors.Is(runErr, context.Canceled) {
			t.Errorf("supervisor exited with %v, want nil or context.Canceled", runErr)
		}
	case <-time.After(settleTimeout):
		t.Error("supervisor did not shut down after context cancellation")
	}
}
