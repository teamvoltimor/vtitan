package main

import (
	"context"
	"encoding/json"
	"fmt"
	"net"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"sync"
	"testing"
	"time"

	"github.com/bufbuild/protovalidate-go"
	"github.com/coder/websocket"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/credentials/insecure"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/types/known/timestamppb"

	navigationdomain "github.com/teamvoltimor/vtitan/platform/backend/domain/navigation"
	navigationmemory "github.com/teamvoltimor/vtitan/platform/backend/domain/navigation/memory"
	robotdomain "github.com/teamvoltimor/vtitan/platform/backend/domain/robot"
	robotmemory "github.com/teamvoltimor/vtitan/platform/backend/domain/robot/memory"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/session"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/session/sqlite"
	simulationdomain "github.com/teamvoltimor/vtitan/platform/backend/domain/simulation"
	simulationmemory "github.com/teamvoltimor/vtitan/platform/backend/domain/simulation/memory"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/telemetry"
	"github.com/teamvoltimor/vtitan/platform/backend/domain/telemetry/memory"
	visiondomain "github.com/teamvoltimor/vtitan/platform/backend/domain/vision"
	visionmemory "github.com/teamvoltimor/vtitan/platform/backend/domain/vision/memory"
	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
	"github.com/teamvoltimor/vtitan/platform/backend/internal/config"
	"github.com/teamvoltimor/vtitan/platform/backend/internal/edge"
	"github.com/teamvoltimor/vtitan/platform/backend/internal/ingest"
)

// testServer bundles the real gRPC ingest server and HTTP edge router, wired
// exactly like run() in main.go (same interceptor chain, same domain service
// construction), but bound to ephemeral addresses and driven directly rather
// than through ListenAndServe/os.Signal — this exercises the actual wire
// transport, not just handlers-in-isolation.
type testServer struct {
	router   http.Handler
	grpcAddr string
}

// newTestServer wires up a full backend instance backed by a real (temp-dir)
// SQLite session store, matching run()'s construction. The gRPC server is
// stopped via t.Cleanup.
func newTestServer(t *testing.T) *testServer {
	t.Helper()

	cfg := &config.Config{
		HistorySize: 360,
		MaxSessions: 20,
		SessionsDir: t.TempDir(),
		DBPath:      filepath.Join(t.TempDir(), "sessions.db"),
		Dev:         true,
	}

	mem := memory.NewMemory(cfg.HistorySize)
	telSvc := telemetry.NewService(mem)

	ctx, cancel := context.WithCancel(context.Background())

	rec, err := sqlite.New(ctx, cfg.DBPath, cfg.SessionsDir, cfg.MaxSessions)
	if err != nil {
		cancel()
		t.Fatalf("sqlite.New: %v", err)
	}
	sessSvc := session.NewService(rec)

	robotCmdSrv := newFakeRobotCommandServer()
	robotSvc := robotdomain.NewService(robotmemory.NewMemory(), robotCmdSrv)
	defaultRobot, err := robotSvc.Create(ctx, robotdomain.CreateRequest{Name: "vtitan"})
	if err != nil {
		cancel()
		t.Fatalf("seed default robot: %v", err)
	}
	navSvc := navigationdomain.NewService(navigationmemory.NewMemory())
	simSvc := simulationdomain.NewService(simulationmemory.NewMemory())
	visSvc := visiondomain.NewService(visionmemory.NewMemory(), mem)

	validator, err := protovalidate.New()
	if err != nil {
		cancel()
		t.Fatalf("protovalidate.New: %v", err)
	}

	grpcSrv := grpc.NewServer(
		grpc.ChainStreamInterceptor(
			streamRecoveryInterceptor(),
			streamValidationInterceptor(validator),
			streamLoggingInterceptor(),
		),
		grpc.ChainUnaryInterceptor(
			unaryRecoveryInterceptor(),
			unaryValidationInterceptor(validator),
			unaryLoggingInterceptor(),
		),
	)
	telemetryv1.RegisterTelemetryIngestServiceServer(grpcSrv, ingest.New(telSvc, sessSvc))

	lc := &net.ListenConfig{}
	lis, err := lc.Listen(ctx, "tcp", "127.0.0.1:0")
	if err != nil {
		cancel()
		t.Fatalf("listen: %v", err)
	}

	router := edge.NewRouter(edge.Services{
		Telemetry:      telSvc,
		Session:        sessSvc,
		Robot:          robotSvc,
		DefaultRobotID: defaultRobot.ID,
		Navigation:     navSvc,
		Simulation:     simSvc,
		Vision:         visSvc,
	}, cfg)

	go func() { _ = grpcSrv.Serve(lis) }()

	t.Cleanup(func() {
		grpcSrv.Stop()
		_ = rec.Close()
		cancel()
	})

	return &testServer{grpcAddr: lis.Addr().String(), router: router}
}

// newFakeRobotCommandServer satisfies robotdomain's command-dispatch
// dependency; the command channel isn't exercised by this test.
type fakeRobotCommandServer struct{}

func newFakeRobotCommandServer() *fakeRobotCommandServer { return &fakeRobotCommandServer{} }

func (*fakeRobotCommandServer) Dispatch(
	_ context.Context, _ string, _ robotdomain.Command,
) (robotdomain.CommandResult, error) {
	return robotdomain.CommandResult{Status: robotdomain.CommandQueued}, nil
}

// grpcClient dials the test server's gRPC ingest endpoint.
func (ts *testServer) grpcClient(t *testing.T) telemetryv1.TelemetryIngestServiceClient {
	t.Helper()
	conn, err := grpc.NewClient(ts.grpcAddr, grpc.WithTransportCredentials(insecure.NewCredentials()))
	if err != nil {
		t.Fatalf("grpc.NewClient: %v", err)
	}
	t.Cleanup(func() { _ = conn.Close() })
	return telemetryv1.NewTelemetryIngestServiceClient(conn)
}

// httpGet issues a GET against the test server's HTTP edge router directly
// (no live listener needed, matching auto-annotator/api's handler-test style).
func (ts *testServer) httpGet(t *testing.T, path string) *httptest.ResponseRecorder {
	t.Helper()
	req := httptest.NewRequestWithContext(context.Background(), http.MethodGet, path, http.NoBody)
	w := httptest.NewRecorder()
	ts.router.ServeHTTP(w, req)
	return w
}

func validSnapshot(missionName string) *telemetryv1.RobotSnapshot {
	return &telemetryv1.RobotSnapshot{
		Timestamp:   timestamppb.Now(),
		MissionName: missionName,
		Metrics: &telemetryv1.TelemetryMetrics{
			Timestamp: timestamppb.Now(),
		},
	}
}

// TestIngestToHTTPEdge_SnapshotRoundTrips ingests a RobotSnapshot over the
// real gRPC transport (through the full interceptor chain) and verifies it
// becomes visible via the HTTP edge's /latest and /history endpoints —
// exercising the actual cross-surface data path a real robot/backend pair
// relies on, not just each handler in isolation.
func TestIngestToHTTPEdge_SnapshotRoundTrips(t *testing.T) {
	ts := newTestServer(t)
	client := ts.grpcClient(t)

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	stream, err := client.StreamSnapshots(ctx)
	if err != nil {
		t.Fatalf("StreamSnapshots: %v", err)
	}
	snapshot := validSnapshot("wro-2026-integration-test")
	if sendErr := stream.Send(&telemetryv1.StreamSnapshotsRequest{Snapshot: snapshot}); sendErr != nil {
		t.Fatalf("stream.Send: %v", sendErr)
	}
	resp, err := stream.CloseAndRecv()
	if err != nil {
		t.Fatalf("stream.CloseAndRecv: %v", err)
	}
	if resp.SnapshotsReceived != 1 {
		t.Fatalf("SnapshotsReceived = %d, want 1", resp.SnapshotsReceived)
	}

	w := ts.httpGet(t, "/v1/telemetry/latest")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/latest status = %d, body = %s", w.Code, w.Body.String())
	}
	var latest struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &latest); err != nil {
		t.Fatalf("unmarshal /latest body: %v", err)
	}
	if latest.MissionName != "wro-2026-integration-test" {
		t.Fatalf("latest.MissionName = %q, want %q", latest.MissionName, "wro-2026-integration-test")
	}

	w = ts.httpGet(t, "/v1/telemetry/history")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/history status = %d, body = %s", w.Code, w.Body.String())
	}
	var history []struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &history); err != nil {
		t.Fatalf("unmarshal /history body: %v", err)
	}
	if len(history) != 1 || history[0].MissionName != "wro-2026-integration-test" {
		t.Fatalf("history = %+v, want one entry with mission_name %q", history, "wro-2026-integration-test")
	}
}

// TestIngestGRPC_InvalidSnapshotMidStream_ReturnsInvalidArgumentAndServerSurvives
// sends a valid frame followed by one that fails protovalidate (empty
// mission_name, min_len=1) partway through a StreamSnapshots call. The
// validatingStream.RecvMsg wrapper runs on every Recv, not just the first
// message, so the server must reject the bad frame with INVALID_ARGUMENT and
// tear down only that stream — a brand new stream on the same server must
// still succeed, proving one bad producer can't wedge the ingest service.
func TestIngestGRPC_InvalidSnapshotMidStream_ReturnsInvalidArgumentAndServerSurvives(t *testing.T) {
	ts := newTestServer(t)
	client := ts.grpcClient(t)

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	stream, err := client.StreamSnapshots(ctx)
	if err != nil {
		t.Fatalf("StreamSnapshots: %v", err)
	}
	firstSend := stream.Send(&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot("valid-first")})
	if firstSend != nil {
		t.Fatalf("stream.Send(valid): %v", firstSend)
	}

	invalid := validSnapshot("")
	// The server-side validation error closes the stream asynchronously, so
	// the exact Send() at which the client observes it can vary; keep
	// sending the invalid frame until Send reports the stream is closed.
	var sendErr error
	for range 10 {
		sendErr = stream.Send(&telemetryv1.StreamSnapshotsRequest{Snapshot: invalid})
		if sendErr != nil {
			break
		}
	}

	_, recvErr := stream.CloseAndRecv()
	if recvErr == nil {
		t.Fatal("CloseAndRecv: expected an error after sending an invalid snapshot, got nil")
	}
	if got := status.Code(recvErr); got != codes.InvalidArgument {
		t.Fatalf(
			"CloseAndRecv error code = %v, want %v (error: %v; last Send error: %v)",
			got,
			codes.InvalidArgument,
			recvErr,
			sendErr,
		)
	}

	// The server must still be healthy: a fresh stream on the same instance
	// should succeed independently of the aborted one above.
	stream2, err := client.StreamSnapshots(ctx)
	if err != nil {
		t.Fatalf("StreamSnapshots (recovery): %v", err)
	}
	if sendErr := stream2.Send(
		&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot("valid-after-recovery")},
	); sendErr != nil {
		t.Fatalf("stream.Send (recovery): %v", sendErr)
	}
	resp, err := stream2.CloseAndRecv()
	if err != nil {
		t.Fatalf("CloseAndRecv (recovery): %v", err)
	}
	if resp.SnapshotsReceived != 1 {
		t.Fatalf("SnapshotsReceived (recovery) = %d, want 1", resp.SnapshotsReceived)
	}

	w := ts.httpGet(t, "/v1/telemetry/latest")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/latest status = %d, body = %s", w.Code, w.Body.String())
	}
	var latest struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &latest); err != nil {
		t.Fatalf("unmarshal /latest body: %v", err)
	}
	if latest.MissionName != "valid-after-recovery" {
		t.Fatalf(
			"latest.MissionName = %q, want %q (the invalid frame must never have been written)",
			latest.MissionName,
			"valid-after-recovery",
		)
	}
}

func validTopicsSnapshot(topicName string) *telemetryv1.TopicsSnapshot {
	return &telemetryv1.TopicsSnapshot{
		Timestamp: timestamppb.Now(),
		Topics: []*telemetryv1.TopicUpdate{
			{
				TopicName:    topicName,
				Timestamp:    timestamppb.Now(),
				UpdateRateHz: 10,
			},
		},
	}
}

// TestIngestGRPC_ConcurrentSnapshotsAndTopicsClients runs multiple
// StreamSnapshots and StreamTopics clients concurrently against the same
// server instance and asserts every read back from /latest, /history and
// /topics is one of the values actually sent — never a torn/mixed read —
// exercising Memory's separate mu/topicsMu locking under real concurrent
// gRPC producers rather than just in-process goroutines.
func TestIngestGRPC_ConcurrentSnapshotsAndTopicsClients(t *testing.T) {
	ts := newTestServer(t)

	const clients = 4
	const framesPerClient = 25

	ctx, cancel := context.WithTimeout(context.Background(), 15*time.Second)
	defer cancel()

	var wg sync.WaitGroup
	for i := range clients {
		wg.Add(2)
		go func(clientIdx int) {
			defer wg.Done()
			client := ts.grpcClient(t)
			stream, err := client.StreamSnapshots(ctx)
			if err != nil {
				t.Errorf("client %d: StreamSnapshots: %v", clientIdx, err)
				return
			}
			for f := range framesPerClient {
				name := fmt.Sprintf("snap-client%d-frame%d", clientIdx, f)
				if sendErr := stream.Send(
					&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot(name)},
				); sendErr != nil {
					t.Errorf("client %d: stream.Send: %v", clientIdx, sendErr)
					return
				}
			}
			if _, err := stream.CloseAndRecv(); err != nil {
				t.Errorf("client %d: CloseAndRecv: %v", clientIdx, err)
			}
		}(i)
		go func(clientIdx int) {
			defer wg.Done()
			client := ts.grpcClient(t)
			stream, err := client.StreamTopics(ctx)
			if err != nil {
				t.Errorf("client %d: StreamTopics: %v", clientIdx, err)
				return
			}
			for f := range framesPerClient {
				name := fmt.Sprintf("topic-client%d-frame%d", clientIdx, f)
				if sendErr := stream.Send(
					&telemetryv1.StreamTopicsRequest{Topics: validTopicsSnapshot(name)},
				); sendErr != nil {
					t.Errorf("client %d: stream.Send: %v", clientIdx, sendErr)
					return
				}
			}
			if _, err := stream.CloseAndRecv(); err != nil {
				t.Errorf("client %d: CloseAndRecv: %v", clientIdx, err)
			}
		}(i)
	}
	wg.Wait()

	w := ts.httpGet(t, "/v1/telemetry/latest")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/latest status = %d, body = %s", w.Code, w.Body.String())
	}
	var latest struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &latest); err != nil {
		t.Fatalf("unmarshal /latest body: %v", err)
	}
	if !strings.HasPrefix(latest.MissionName, "snap-client") {
		t.Fatalf(
			"latest.MissionName = %q, does not look like a value any client actually sent (corrupted read?)",
			latest.MissionName,
		)
	}

	// historyDefaultLimit (60) is smaller than clients*framesPerClient (100)
	// -- explicit ?limit= is required to read back every frame this test wrote.
	w = ts.httpGet(t, fmt.Sprintf("/v1/telemetry/history?limit=%d", clients*framesPerClient))
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/history status = %d, body = %s", w.Code, w.Body.String())
	}
	var history []struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &history); err != nil {
		t.Fatalf("unmarshal /history body: %v", err)
	}
	if len(history) != clients*framesPerClient {
		t.Fatalf(
			"history length = %d, want %d (dropped or duplicated frames under concurrency)",
			len(history),
			clients*framesPerClient,
		)
	}
	for _, h := range history {
		if !strings.HasPrefix(h.MissionName, "snap-client") {
			t.Fatalf(
				"history entry MissionName = %q, does not look like a value any client actually sent (corrupted read?)",
				h.MissionName,
			)
		}
	}

	w = ts.httpGet(t, "/v1/telemetry/topics")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/topics status = %d, body = %s", w.Code, w.Body.String())
	}
	var topics struct {
		Topics []struct {
			TopicName string `json:"topic_name"`
		} `json:"topics"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &topics); err != nil {
		t.Fatalf("unmarshal /topics body: %v", err)
	}
	if len(topics.Topics) != 1 {
		t.Fatalf("topics.Topics = %+v, want exactly 1 (one topic per TopicsSnapshot payload)", topics.Topics)
	}
	if !strings.HasPrefix(topics.Topics[0].TopicName, "topic-client") {
		t.Fatalf(
			"topics.Topics[0].TopicName = %q, does not look like a value any client actually sent (corrupted read?)",
			topics.Topics[0].TopicName,
		)
	}
}

// TestSessionPersistence_RoundTrips ingests a sequence of snapshots, then
// verifies they're independently recoverable from the real SQLite-indexed
// JSONL session store — not just from the in-memory ring buffer /history
// already exercises — by listing sessions via the HTTP edge and replaying
// the discovered session ID.
func TestSessionPersistence_RoundTrips(t *testing.T) {
	ts := newTestServer(t)
	client := ts.grpcClient(t)

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	stream, err := client.StreamSnapshots(ctx)
	if err != nil {
		t.Fatalf("StreamSnapshots: %v", err)
	}
	missionNames := []string{"session-frame-0", "session-frame-1", "session-frame-2"}
	for _, name := range missionNames {
		if sendErr := stream.Send(&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot(name)}); sendErr != nil {
			t.Fatalf("stream.Send(%q): %v", name, sendErr)
		}
	}
	if _, err := stream.CloseAndRecv(); err != nil {
		t.Fatalf("CloseAndRecv: %v", err)
	}

	w := ts.httpGet(t, "/v1/telemetry/sessions")
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/sessions status = %d, body = %s", w.Code, w.Body.String())
	}
	var sessions []struct {
		SessionID  string `json:"session_id"`
		EntryCount int    `json:"entry_count"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &sessions); err != nil {
		t.Fatalf("unmarshal /sessions body: %v", err)
	}
	// newTestServer opens exactly one Recorder (= one session) per test.
	if len(sessions) != 1 {
		t.Fatalf("sessions = %+v, want exactly 1", sessions)
	}
	if sessions[0].EntryCount != len(missionNames) {
		t.Fatalf("sessions[0].EntryCount = %d, want %d", sessions[0].EntryCount, len(missionNames))
	}

	w = ts.httpGet(t, "/v1/telemetry/sessions/"+sessions[0].SessionID)
	if w.Code != http.StatusOK {
		t.Fatalf("GET /v1/telemetry/sessions/%s status = %d, body = %s", sessions[0].SessionID, w.Code, w.Body.String())
	}
	var replay []struct {
		MissionName string `json:"mission_name"`
	}
	if err := json.Unmarshal(w.Body.Bytes(), &replay); err != nil {
		t.Fatalf("unmarshal session replay body: %v", err)
	}
	if len(replay) != len(missionNames) {
		t.Fatalf("replay length = %d, want %d", len(replay), len(missionNames))
	}
	for i, name := range missionNames {
		if replay[i].MissionName != name {
			t.Fatalf(
				"replay[%d].MissionName = %q, want %q (order must match ingestion order)",
				i,
				replay[i].MissionName,
				name,
			)
		}
	}
}

// httpServer starts a real net/http listener in front of ts.router — needed
// for the WebSocket test below, since httptest.ResponseRecorder (used by
// ts.httpGet) can't hijack a connection for the Upgrade handshake the way a
// live listener can.
func (ts *testServer) httpServer(t *testing.T) *httptest.Server {
	t.Helper()
	srv := httptest.NewServer(ts.router)
	t.Cleanup(srv.Close)
	return srv
}

// TestWebSocketBroadcast_ReceivesIngestedSnapshot dials the real /v1/telemetry/ws
// endpoint, ingests a snapshot over the real gRPC transport, and asserts the
// WS subscriber receives it — exercising the full producer(gRPC)->Memory->
// wsManager.Subscribe->consumer(WS) fan-out path end to end.
func TestWebSocketBroadcast_ReceivesIngestedSnapshot(t *testing.T) {
	ts := newTestServer(t)
	httpSrv := ts.httpServer(t)

	ctx, cancel := context.WithTimeout(context.Background(), 10*time.Second)
	defer cancel()

	wsURL := "ws" + strings.TrimPrefix(httpSrv.URL, "http") + "/v1/telemetry/ws"
	//nolint:bodyclose // coder/websocket's Dial nils resp.Body after taking
	// over the hijacked connection ("you never need to close resp.Body
	// yourself" per its doc comment) -- closing it here would nil-panic.
	conn, _, err := websocket.Dial(ctx, wsURL, nil)
	if err != nil {
		t.Fatalf("websocket.Dial: %v", err)
	}
	defer func() { _ = conn.CloseNow() }()

	client := ts.grpcClient(t)
	stream, err := client.StreamSnapshots(ctx)
	if err != nil {
		t.Fatalf("StreamSnapshots: %v", err)
	}
	// wsManager.Subscribe() registers only after the Accept handshake
	// completes; a snapshot sent before that registration would never reach
	// this subscriber. Give the handshake a moment to land before ingesting
	// — the test still fails (via the read below timing out) if the
	// broadcast path itself is broken, this just avoids a race against the
	// server's own goroutine scheduling.
	time.Sleep(50 * time.Millisecond)
	if sendErr := stream.Send(
		&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot("ws-broadcast-test")},
	); sendErr != nil {
		t.Fatalf("stream.Send: %v", sendErr)
	}

	_, data, err := conn.Read(ctx)
	if err != nil {
		t.Fatalf("conn.Read: %v", err)
	}
	var snap telemetryv1.RobotSnapshot
	if err := protojson.Unmarshal(data, &snap); err != nil {
		t.Fatalf("protojson.Unmarshal WS frame: %v", err)
	}
	if snap.GetMissionName() != "ws-broadcast-test" {
		t.Fatalf("WS-received MissionName = %q, want %q", snap.GetMissionName(), "ws-broadcast-test")
	}

	if _, err := stream.CloseAndRecv(); err != nil {
		t.Fatalf("CloseAndRecv: %v", err)
	}
}
