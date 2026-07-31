package main

import (
	"context"
	"encoding/json"
	"net"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"testing"
	"time"

	"github.com/bufbuild/protovalidate-go"
	"google.golang.org/grpc"
	"google.golang.org/grpc/credentials/insecure"
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
	grpcAddr string
	router   http.Handler
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

	lis, err := net.Listen("tcp", "127.0.0.1:0")
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
	req := httptest.NewRequest(http.MethodGet, path, nil)
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
	if err := stream.Send(&telemetryv1.StreamSnapshotsRequest{Snapshot: validSnapshot("wro-2026-integration-test")}); err != nil {
		t.Fatalf("stream.Send: %v", err)
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
