package nats_test

import (
	"context"
	"testing"
	"time"

	natssrv "github.com/nats-io/nats-server/v2/server"
	natstest "github.com/nats-io/nats-server/v2/test"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"

	actuationv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/actuation/v1"
)

const (
	serverReadyTimeout = 2 * time.Second
	serverPollInterval = 10 * time.Millisecond
)

// startTestServer starts an in-process, ephemeral-port nats-server for the
// duration of the test, per go-architect §9's "in-process, no Docker" NATS
// integration test pattern -- nats-server is itself a Go library, no
// external process/container needed.
func startTestServer(t *testing.T) string {
	t.Helper()

	opts := natstest.DefaultTestOptions
	opts.Port = -1 // -1 means "pick a random free port", avoids collisions between parallel tests
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

func TestPublisherSubscriber_RoundTrip(t *testing.T) {
	t.Parallel()

	url := startTestServer(t)
	cfg := nats.DefaultConfig(url, "test-round-trip")
	conn, err := nats.Connect(cfg)
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)

	const subject = "vtitan.actuation.v1.ackermann_cmd.test"

	sub, err := nats.NewSubscriber(conn, subject, func() *actuationv1.AckermannCmd {
		return &actuationv1.AckermannCmd{}
	})
	if err != nil {
		t.Fatalf("NewSubscriber() error = %v, want nil", err)
	}
	t.Cleanup(func() { _ = sub.Close() })

	pub := nats.NewPublisher[*actuationv1.AckermannCmd](conn, subject)

	want := &actuationv1.AckermannCmd{
		Stamp:         timestamppb.Now(),
		FrameId:       "base_link",
		SteeringAngle: 0.25,
		Speed:         0.5,
	}
	if err = pub.Publish(want); err != nil {
		t.Fatalf("Publish() error = %v, want nil", err)
	}

	ctx, cancel := context.WithTimeout(context.Background(), 2*time.Second)
	defer cancel()

	got, err := sub.Read(ctx)
	if err != nil {
		t.Fatalf("Read() error = %v, want nil", err)
	}

	if got.GetFrameId() != want.GetFrameId() ||
		got.GetSteeringAngle() != want.GetSteeringAngle() ||
		got.GetSpeed() != want.GetSpeed() {
		t.Errorf("Read() = %+v, want %+v", got, want)
	}
}

func TestSubscriber_Read_RespectsContextCancellation(t *testing.T) {
	t.Parallel()

	url := startTestServer(t)
	cfg := nats.DefaultConfig(url, "test-cancel")
	conn, err := nats.Connect(cfg)
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)

	sub, err := nats.NewSubscriber(conn, "vtitan.actuation.v1.ackermann_cmd.nobody-publishes-here",
		func() *actuationv1.AckermannCmd { return &actuationv1.AckermannCmd{} })
	if err != nil {
		t.Fatalf("NewSubscriber() error = %v, want nil", err)
	}
	t.Cleanup(func() { _ = sub.Close() })

	ctx, cancel := context.WithTimeout(context.Background(), 50*time.Millisecond)
	defer cancel()

	if _, err = sub.Read(ctx); err == nil {
		t.Fatal("Read() with no publisher and a short timeout: got nil error, want a timeout error")
	}
}

func TestConnect_InvalidConfig(t *testing.T) {
	t.Parallel()

	if _, err := nats.Connect(nats.Config{}); err == nil {
		t.Fatal("Connect(Config{}) with no URL/Name: got nil error, want a validation error")
	}
}
