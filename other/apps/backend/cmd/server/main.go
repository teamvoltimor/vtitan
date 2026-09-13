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
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/reflection"
	"google.golang.org/grpc/status"
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
	telemetryv1.RegisterRobotCommandServiceServer(grpcSrv, robotCmdSrv)
	if cfg.Dev {
		reflection.Register(grpcSrv)
	}

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

// streamRecoveryInterceptor catches panics in streaming handlers and returns INTERNAL.
func streamRecoveryInterceptor() grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) (err error) {
		defer func() {
			if r := recover(); r != nil {
				slog.Error("gRPC stream panic", "panic", r, "method", info.FullMethod)
				err = status.Errorf(codes.Internal, "internal server error")
			}
		}()
		return handler(srv, ss)
	}
}

// streamValidationInterceptor validates the first message of a client-stream via protovalidate.
// Per-message validation is done inside each streaming handler (interceptors run once per RPC).
func streamValidationInterceptor(v protovalidate.Validator) grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) error {
		return handler(srv, &validatingStream{ServerStream: ss, v: v})
	}
}

type validatingStream struct {
	grpc.ServerStream
	v protovalidate.Validator
}

func (s *validatingStream) RecvMsg(m any) error {
	if err := s.ServerStream.RecvMsg(m); err != nil {
		return err
	}
	if msg, ok := m.(proto.Message); ok {
		if err := s.v.Validate(msg); err != nil {
			return status.Errorf(codes.InvalidArgument, "validation: %v", err)
		}
	}
	return nil
}

// unaryRecoveryInterceptor catches panics in unary handlers and returns INTERNAL.
func unaryRecoveryInterceptor() grpc.UnaryServerInterceptor {
	return func(ctx context.Context, req any, info *grpc.UnaryServerInfo, handler grpc.UnaryHandler) (resp any, err error) {
		defer func() {
			if r := recover(); r != nil {
				slog.Error("gRPC unary panic", "panic", r, "method", info.FullMethod)
				err = status.Errorf(codes.Internal, "internal server error")
			}
		}()
		return handler(ctx, req)
	}
}

// unaryValidationInterceptor validates the request message via protovalidate.
func unaryValidationInterceptor(v protovalidate.Validator) grpc.UnaryServerInterceptor {
	return func(ctx context.Context, req any, info *grpc.UnaryServerInfo, handler grpc.UnaryHandler) (any, error) {
		if msg, ok := req.(proto.Message); ok {
			if err := v.Validate(msg); err != nil {
				return nil, status.Errorf(codes.InvalidArgument, "validation: %v", err)
			}
		}
		return handler(ctx, req)
	}
}

// unaryLoggingInterceptor logs each unary RPC with method and outcome.
func unaryLoggingInterceptor() grpc.UnaryServerInterceptor {
	return func(ctx context.Context, req any, info *grpc.UnaryServerInfo, handler grpc.UnaryHandler) (any, error) {
		resp, err := handler(ctx, req)
		if err != nil {
			slog.Warn("gRPC unary error", "method", info.FullMethod, "error", err)
		} else {
			slog.Info("gRPC unary completed", "method", info.FullMethod)
		}
		return resp, err
	}
}

// streamLoggingInterceptor logs each streaming RPC with method and outcome.
func streamLoggingInterceptor() grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) error {
		slog.Info("gRPC stream started", "method", info.FullMethod)
		err := handler(srv, ss)
		if err != nil {
			slog.Warn("gRPC stream error", "method", info.FullMethod, "error", err)
		} else {
			slog.Info("gRPC stream completed", "method", info.FullMethod)
		}
		return err
	}
}
