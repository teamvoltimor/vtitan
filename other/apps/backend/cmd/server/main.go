package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"log/slog"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/bufbuild/protovalidate-go"
	"github.com/ralvarezdev/grpckit"
	"google.golang.org/protobuf/proto"

	navigationdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/navigation"
	navigationmemory "github.com/teamvoltimor/vtitan/apps/backend/domain/navigation/memory"
	robotdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/robot"
	robotmemory "github.com/teamvoltimor/vtitan/apps/backend/domain/robot/memory"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/session"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/session/sqlite"
	simulationdomain "github.com/teamvoltimor/vtitan/apps/backend/domain/simulation"
	simulationmemory "github.com/teamvoltimor/vtitan/apps/backend/domain/simulation/memory"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/telemetry"
	"github.com/teamvoltimor/vtitan/apps/backend/domain/telemetry/memory"
	visiondomain "github.com/teamvoltimor/vtitan/apps/backend/domain/vision"
	visionmemory "github.com/teamvoltimor/vtitan/apps/backend/domain/vision/memory"
	telemetryv1 "github.com/teamvoltimor/vtitan/apps/backend/gen/telemetry/v1"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/config"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/edge"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/ingest"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/robotcmd"
	"github.com/teamvoltimor/vtitan/apps/backend/internal/sim"
)

const (
	// defaultRobotName seeds the singleton robot this single-robot project's
	// legacy telemetry speed-config endpoint delegates to.
	defaultRobotName      = "vtitan"
	shutdownTimeout       = 10 * time.Second
	httpReadHeaderTimeout = 10 * time.Second
)

func main() {
	simMode := flag.Bool("sim", false, "run with synthetic telemetry generator (dev only)")
	flag.Parse()

	slog.SetDefault(slog.New(slog.NewJSONHandler(os.Stdout, nil)))

	if err := run(*simMode); err != nil {
		slog.Error("server error", "error", err)
		os.Exit(1)
	}
}

func run(simMode bool) error {
	cfg := config.Load()

	mem := memory.NewMemory(cfg.HistorySize)
	telSvc := telemetry.NewService(mem)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	rec, err := sqlite.New(ctx, cfg.DBPath, cfg.SessionsDir, cfg.MaxSessions)
	if err != nil {
		return fmt.Errorf("init recorder: %w", err)
	}
	defer rec.Close()
	sessSvc := session.NewService(rec)

	robotCmdSrv := robotcmd.New()
	robotSvc := robotdomain.NewService(robotmemory.NewMemory(), robotCmdSrv)
	defaultRobot, err := robotSvc.Create(ctx, robotdomain.CreateRequest{Name: defaultRobotName})
	if err != nil {
		return fmt.Errorf("seed default robot: %w", err)
	}
	navSvc := navigationdomain.NewService(navigationmemory.NewMemory())
	simSvc := simulationdomain.NewService(simulationmemory.NewMemory())
	visSvc := visiondomain.NewService(visionmemory.NewMemory(), mem)

	validator, err := protovalidate.New()
	if err != nil {
		return fmt.Errorf("init protovalidate: %w", err)
	}

	grpcSrv := grpckit.NewServer(grpckit.ServerConfig{
		Validate: func(m proto.Message) error {
			return validator.Validate(m)
		},
		EnableReflection: cfg.Dev,
	})
	telemetryv1.RegisterTelemetryIngestServiceServer(grpcSrv, ingest.New(telSvc, sessSvc))
	telemetryv1.RegisterRobotCommandServiceServer(grpcSrv, robotCmdSrv)

	lc := &net.ListenConfig{}
	lis, err := lc.Listen(ctx, "tcp", cfg.GRPCAddr)
	if err != nil {
		return fmt.Errorf("gRPC listen %s: %w", cfg.GRPCAddr, err)
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
	httpSrv := &http.Server{
		Addr:              cfg.HTTPAddr,
		Handler:           router,
		ReadHeaderTimeout: httpReadHeaderTimeout,
	}

	if simMode {
		g := sim.New(telSvc)
		go g.Run(ctx, cfg.SimInterval)
	}

	go func() {
		slog.Info("gRPC ingest listening", "addr", cfg.GRPCAddr)
		if err := grpcSrv.Serve(lis); err != nil {
			slog.Error("gRPC serve", "error", err)
		}
	}()

	go func() {
		slog.Info("HTTP edge listening", "addr", cfg.HTTPAddr)
		if err := httpSrv.ListenAndServe(); !errors.Is(err, http.ErrServerClosed) {
			slog.Error("HTTP serve", "error", err)
		}
	}()

	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	slog.Info("shutting down")
	cancel()
	grpcSrv.GracefulStop()
	shutCtx, shutCancel := context.WithTimeout(context.Background(), shutdownTimeout)
	defer shutCancel()
	_ = httpSrv.Shutdown(shutCtx)
	return nil
}
