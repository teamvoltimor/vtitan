package picolink_test

import (
	"context"
	"log/slog"
	"net"
	"testing"
	"time"

	natstest "github.com/nats-io/nats-server/v2/test"

	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// serverReadyTimeout bounds the embedded nats-server's start.
const serverReadyTimeout = 5 * time.Second

// Serve uses the Zero's subjects: a command published on ackermann_cmd
// reaches the board, and the board's Status and Odometry come back on
// motor_status and joint_states.
func TestServe_UsesTheZerosSubjects(t *testing.T) {
	t.Parallel()

	opts := natstest.DefaultTestOptions
	opts.Port = -1
	srv := natstest.RunServer(&opts)
	t.Cleanup(func() {
		srv.Shutdown()
		srv.WaitForShutdown()
	})
	if !srv.ReadyForConnections(serverReadyTimeout) {
		t.Fatal("nats-server did not become ready")
	}
	url := srv.ClientURL()

	ctx, cancel := context.WithCancel(t.Context())
	defer cancel()

	conn, err := nats.Connect(ctx, nats.DefaultConfig(url, "picolink-test"))
	if err != nil {
		t.Fatalf("connect: %v", err)
	}
	defer conn.Close()
	statusSub, err := nats.NewSubscriber[actuationv1.MotorStatus](conn, actuationv1.MotorStatusSubject)
	if err != nil {
		t.Fatal(err)
	}
	jointSub, err := nats.NewSubscriber[actuationv1.JointStates](conn, actuationv1.JointStatesSubject)
	if err != nil {
		t.Fatal(err)
	}
	cmdPub := nats.NewPublisher[*actuationv1.AckermannCmd](conn, actuationv1.AckermannCmdSubject)

	hostEnd, boardEnd := net.Pipe()
	board := newFakeBoard(boardEnd)
	done := make(chan error, 1)
	go func() {
		done <- picolink.Serve(ctx, picolink.Config{
			NATS: nats.DefaultConfig(url, "picolink"),
			Session: picolink.SessionConfig{
				Board:   sampleBoard,
				Encoder: &picolink.EncoderParams{CountsPerRev: 60},
			},
		}, slog.New(slog.DiscardHandler), hostEnd)
	}()
	t.Cleanup(func() {
		cancel()
		<-done
		_ = hostEnd.Close()
		_ = boardEnd.Close()
	})

	// The board's Hello is answered only once Serve is subscribed, so this
	// also orders the command publish after the subscription exists.
	board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 3,
	}})
	board.expect(t, boardlink.TypeConfig)

	if err = cmdPub.Publish(&actuationv1.AckermannCmd{Speed: 0.3, SteeringAngle: 0.1}); err != nil {
		t.Fatal(err)
	}
	if got := board.expect(t, boardlink.TypeCommand).Command; got.SpeedMPS != 0.3 || got.SteeringAngleRad != 0.1 {
		t.Errorf("Command = %+v", got)
	}

	board.send(t, boardlink.Packet{Type: boardlink.TypeStatus, Status: boardlink.Status{
		State: boardlink.StateRunning, Duty: 0.09, CommandAgeMS: 4,
	}})
	readCtx, readCancel := context.WithTimeout(ctx, waitTimeout)
	defer readCancel()
	st, err := statusSub.Read(readCtx)
	if err != nil {
		t.Fatalf("no MotorStatus on %s: %v", actuationv1.MotorStatusSubject, err)
	}
	if st.GetState() != actuationv1.MotorStatus_STATE_RUNNING || st.GetDutyCycle() != 0.09 {
		t.Errorf("MotorStatus = %v", st)
	}

	board.send(t, boardlink.Packet{Type: boardlink.TypeOdometry, Odometry: boardlink.Odometry{
		BoardTimeUS: 1, Counts: 30,
	}})
	js, err := jointSub.Read(readCtx)
	if err != nil {
		t.Fatalf("no JointStates on %s: %v", actuationv1.JointStatesSubject, err)
	}
	if js.GetName()[0] != actuationv1.DriveJoint || js.GetPosition()[0] <= 0 {
		t.Errorf("JointStates = %v", js)
	}
}
