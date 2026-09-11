package nats_test

import (
	"context"
	"fmt"
	"net"
	"testing"
	"time"

	natssrv "github.com/nats-io/nats-server/v2/server"
	natstest "github.com/nats-io/nats-server/v2/test"
	natsgo "github.com/nats-io/nats.go"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
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
	conn, err := nats.Connect(context.Background(), cfg)
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)

	const subject = "vtitan.actuation.v1.ackermann_cmd.test"

	sub, err := nats.NewSubscriber[actuationv1.AckermannCmd](conn, subject)
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
	conn, err := nats.Connect(context.Background(), cfg)
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)

	sub, err := nats.NewSubscriber[actuationv1.AckermannCmd](
		conn, "vtitan.actuation.v1.ackermann_cmd.nobody-publishes-here")
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

	if _, err := nats.Connect(context.Background(), nats.Config{}); err == nil {
		t.Fatal("Connect(Config{}) with no URL/Name: got nil error, want a validation error")
	}
}

// TestConnect_RetriesInitialDialUntilServerReady proves the cold-boot case: a
// client that calls Connect before nats-server is listening still connects once
// the server comes up, instead of failing its initial dial. This is what makes
// a Pi 5 reboot (where both boards cold-boot over the USB-gadget link and
// nats-server can take tens of seconds) survivable without ordering the binary
// start after the server.
func TestConnect_RetriesInitialDialUntilServerReady(t *testing.T) {
	t.Parallel()

	// Pick a free port, then start the server late (after Connect is already
	// retrying) to simulate the server not being up yet.
	listener, err := (&net.ListenConfig{}).Listen(t.Context(), "tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("reserving a port: %v", err)
	}
	tcpAddr, ok := listener.Addr().(*net.TCPAddr)
	if !ok {
		t.Fatalf("listener address is %T, want *net.TCPAddr", listener.Addr())
	}
	port := tcpAddr.Port
	// Release the port so nats-server can bind it; we kept it only to learn a
	// free number.
	if err = listener.Close(); err != nil {
		t.Fatalf("closing probe listener: %v", err)
	}
	url := fmt.Sprintf("nats://127.0.0.1:%d", port)

	cfg := nats.DefaultConfig(url, "test-late-server")
	cfg.InitialConnectAttempts = -1 // unlimited while the test context is alive

	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()

	connected := make(chan *natsgo.Conn, 1)
	go func() {
		conn, cerr := nats.Connect(ctx, cfg)
		if cerr != nil {
			return
		}
		connected <- conn
	}()

	// Let Connect begin retrying, then bring the server up.
	time.Sleep(300 * time.Millisecond)
	srvOpts := natstest.DefaultTestOptions
	srvOpts.Host = "127.0.0.1"
	srvOpts.Port = port
	srv := natstest.RunServer(&srvOpts)
	defer func() {
		srv.Shutdown()
		srv.WaitForShutdown()
	}()

	select {
	case conn := <-connected:
		conn.Close()
	case <-ctx.Done():
		t.Fatal("Connect never succeeded even after the server came up")
	}
}

// TestConnect_InvalidConfigBoundedRetries proves that a server that is simply
// unreachable (not just not-yet-up) respects a finite InitialConnectAttempts
// rather than looping forever.
func TestConnect_InvalidConfigBoundedRetries(t *testing.T) {
	t.Parallel()

	cfg := nats.DefaultConfig("nats://127.0.0.1:1", "test-unreachable")
	cfg.InitialConnectAttempts = 3
	cfg.ReconnectWait = 10 * time.Millisecond

	start := time.Now()
	_, err := nats.Connect(context.Background(), cfg)
	elapsed := time.Since(start)
	if err == nil {
		t.Fatal("Connect to an unreachable port: got nil error, want error")
	}
	if elapsed > 5*time.Second {
		t.Errorf("Connect retried for %v with InitialConnectAttempts=3; want bounded by ~3*ReconnectWait", elapsed)
	}
}
