package main

import (
	"context"
	"encoding/json"
	"log/slog"
	"net"
	"net/http"
	"testing"
	"time"

	"github.com/coder/websocket"
	natstest "github.com/nats-io/nats-server/v2/test"
	natsgo "github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/types/known/timestamppb"

	"github.com/teamvoltimor/vtitan/src/go/internal/cmdkit"
	"github.com/teamvoltimor/vtitan/src/go/internal/foxglove/foxglovetest"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

const testTimeout = 10 * time.Second

func startTestNATS(t *testing.T) string {
	t.Helper()
	opts := natstest.DefaultTestOptions
	opts.Port = -1
	srv := natstest.RunServer(&opts)
	t.Cleanup(func() {
		srv.Shutdown()
		srv.WaitForShutdown()
	})
	deadline := time.Now().Add(testTimeout)
	for !srv.ReadyForConnections(10 * time.Millisecond) {
		if time.Now().After(deadline) {
			t.Fatal("nats-server did not become ready in time")
		}
	}
	return srv.ClientURL()
}

// freeHTTPAddr picks a currently-free TCP port on loopback by binding to
// :0 and immediately releasing it -- the standard, small-race-accepted
// pattern for handing an HTTP server under test a real, unused address
// (run's httpServer.ListenAndServe binds its own listener internally, so
// there is no injectable-listener seam to avoid this).
func freeHTTPAddr(ctx context.Context, t *testing.T) string {
	t.Helper()
	var lc net.ListenConfig
	l, err := lc.Listen(ctx, "tcp", "127.0.0.1:0")
	if err != nil {
		t.Fatalf("net.Listen: %v", err)
	}
	addr := l.Addr().String()
	l.Close()
	return addr
}

// TestRun_BridgesANATSMessageToAFoxgloveClient is the end-to-end contract
// for the whole binary: publish one real message on a real NATS subject,
// and a real Foxglove-protocol websocket client (mirroring what Foxglove
// Studio does) must receive it, decoded correctly, over the bridge's HTTP
// server -- not just the isolated internal/foxglove protocol logic, but
// this command's own wiring (bridgeAllSubjects registering all subjects,
// the NATS subscription loop, the HTTP server lifecycle).
func TestRun_BridgesANATSMessageToAFoxgloveClient(t *testing.T) {
	t.Parallel()

	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	natsURL := startTestNATS(t)
	httpAddr := freeHTTPAddr(ctx, t)

	logger := slog.New(slog.DiscardHandler)
	cfg := cliConfig{Common: cmdkit.Common{NATSURL: natsURL, NodeName: "test-bridge"}, httpAddr: httpAddr}

	runErrCh := make(chan error, 1)
	go func() { runErrCh <- run(ctx, logger, cfg) }()

	wsURL := "ws://" + httpAddr
	conn := dialFoxglove(ctx, t, wsURL)

	var info map[string]any
	readJSONFrame(ctx, t, conn, &info)
	if info["op"] != "serverInfo" {
		t.Fatalf(`first message op = %v, want "serverInfo"`, info["op"])
	}

	var advertise foxglovetest.AdvertiseResponse
	readJSONFrame(ctx, t, conn, &advertise)
	if advertise.Op != "advertise" {
		t.Fatalf(`second message op = %v, want "advertise"`, advertise.Op)
	}
	var imuChannelID uint32
	found := false
	for _, ch := range advertise.Channels {
		if ch.Topic == sensorv1.ImuSubject {
			imuChannelID, found = ch.ID, true
			break
		}
	}
	if !found {
		t.Fatalf("advertise.channels = %+v, want %s present", advertise.Channels, sensorv1.ImuSubject)
	}

	const subscriptionID uint32 = 7
	subMsg, _ := json.Marshal(map[string]any{
		"op": "subscribe",
		"subscriptions": []map[string]any{
			{"id": subscriptionID, "channelId": imuChannelID},
		},
	})
	if err := conn.Write(ctx, websocket.MessageText, subMsg); err != nil {
		t.Fatalf("conn.Write(subscribe): %v", err)
	}

	nc, err := natsgo.Connect(natsURL)
	if err != nil {
		t.Fatalf("natsgo.Connect: %v", err)
	}
	defer nc.Close()

	want := &sensorv1.Imu{
		Stamp:       timestamppb.Now(),
		Orientation: &sensorv1.Quaternion{X: 0.5, Y: 0, Z: 0, W: 0.866},
	}
	wireBytes, err := proto.Marshal(want)
	if err != nil {
		t.Fatalf("proto.Marshal(want): %v", err)
	}

	resultCh := make(chan foxglovetest.ReadResult, 1)
	go func() {
		kind, data, readErr := conn.Read(ctx)
		resultCh <- foxglovetest.ReadResult{Kind: kind, Data: data, Err: readErr}
	}()

	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()
	var frame []byte
	for frame == nil {
		select {
		case res := <-resultCh:
			if res.Err != nil {
				t.Fatalf("conn.Read: %v", res.Err)
			}
			if res.Kind != websocket.MessageBinary {
				t.Fatalf("conn.Read: kind = %v, want MessageBinary", res.Kind)
			}
			frame = res.Data
		case <-ticker.C:
			if pubErr := nc.Publish(sensorv1.ImuSubject, wireBytes); pubErr != nil {
				t.Fatalf("nc.Publish: %v", pubErr)
			}
		case <-ctx.Done():
			t.Fatal("never received a Message Data frame for the published Imu message")
		}
	}

	got := &sensorv1.Imu{}
	if err = proto.Unmarshal(frame[13:], got); err != nil {
		t.Fatalf("proto.Unmarshal(frame payload): %v", err)
	}
	if got.GetOrientation().GetX() != 0.5 || got.GetOrientation().GetW() != 0.866 {
		t.Errorf("decoded Imu = %+v, want orientation matching %+v", got, want.Orientation)
	}

	conn.Close(websocket.StatusNormalClosure, "")
	cancel()
	select {
	case runErr := <-runErrCh:
		if runErr != nil {
			t.Errorf("run() returned %v, want nil after context cancellation", runErr)
		}
	case <-time.After(testTimeout):
		t.Error("run() never returned after context cancellation")
	}
}

func dialFoxglove(ctx context.Context, t *testing.T, wsURL string) *websocket.Conn {
	t.Helper()
	deadline := time.Now().Add(testTimeout)
	var lastErr error
	for time.Now().Before(deadline) {
		conn, resp, err := websocket.Dial(ctx, wsURL, &websocket.DialOptions{
			Subprotocols: []string{"foxglove.websocket.v1"},
			HTTPClient:   &http.Client{Timeout: 500 * time.Millisecond},
		})
		if resp != nil && resp.Body != nil {
			if closeErr := resp.Body.Close(); closeErr != nil {
				t.Logf("websocket.Dial: closing handshake response body: %v", closeErr)
			}
		}
		if err == nil {
			// The default 32KiB per-message read limit is comfortably
			// enough for one channel's advertise but not all 17 registered
			// subjects' schemas (each a full transitive FileDescriptorSet)
			// in a single "advertise" message -- a real Foxglove Studio
			// client raises this itself; mirror that here rather than
			// under-registering channels to fit an artificial test limit.
			conn.SetReadLimit(8 << 20)
			t.Cleanup(func() { conn.Close(websocket.StatusNormalClosure, "") })
			return conn
		}
		lastErr = err
		time.Sleep(20 * time.Millisecond) // the http server may not have started listening yet
	}
	t.Fatalf("websocket.Dial: %v", lastErr)
	return nil
}

func readJSONFrame(ctx context.Context, t *testing.T, conn *websocket.Conn, v any) {
	t.Helper()
	kind, data, err := conn.Read(ctx)
	if err != nil {
		t.Fatalf("conn.Read: %v", err)
	}
	if kind != websocket.MessageText {
		t.Fatalf("conn.Read: kind = %v, want MessageText", kind)
	}
	if err = json.Unmarshal(data, v); err != nil {
		t.Fatalf("json.Unmarshal(%s): %v", data, err)
	}
}
