package main

import (
	"context"
	"errors"
	"flag"
	"fmt"
	"net"
	"net/http"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/bufbuild/protovalidate-go"
	"go.uber.org/zap"
	"google.golang.org/grpc"
	"google.golang.org/grpc/codes"
	"google.golang.org/grpc/reflection"
	"google.golang.org/grpc/status"
	"google.golang.org/protobuf/proto"

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
	"github.com/teamvoltimor/vtitan/platform/backend/internal/sim"
)

// defaultRobotName seeds the singleton robot this single-robot project's
// legacy telemetry speed-config endpoint delegates to.
const defaultRobotName = "vtitan"

const (
	shutdownTimeout       = 10 * time.Second
	httpReadHeaderTimeout = 10 * time.Second
)

func main() {
	simMode := flag.Bool("sim", false, "run with synthetic telemetry generator (dev only)")
	flag.Parse()

	log, err := zap.NewProduction()
	if err != nil {
		panic(err)
	}

	if err := run(log, *simMode); err != nil {
		log.Error("server error", zap.Error(err))
		_ = log.Sync()
		os.Exit(1)
	}
	_ = log.Sync()
}

func run(log *zap.Logger, simMode bool) error {
	cfg := config.Load()

	mem := memory.NewMemory(cfg.HistorySize)
	telSvc := telemetry.NewService(mem)

	ctx, cancel := context.WithCancel(context.Background())
	defer cancel()

	rec, err := sqlite.New(ctx, cfg.DBPath, cfg.SessionsDir, cfg.MaxSessions, log)
	if err != nil {
		return fmt.Errorf("init recorder: %w", err)
	}
	defer rec.Close()
	sessSvc := session.NewService(rec)

	robotSvc := robotdomain.NewService(robotmemory.NewMemory())
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
			streamRecoveryInterceptor(log),
			streamValidationInterceptor(validator),
			streamLoggingInterceptor(log),
		),
	)
	telemetryv1.RegisterTelemetryIngestServiceServer(grpcSrv, ingest.New(telSvc, sessSvc, log))
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
	}, cfg, log)
	httpSrv := &http.Server{
		Addr:              cfg.HTTPAddr,
		Handler:           router,
		ReadHeaderTimeout: httpReadHeaderTimeout,
	}

	if simMode {
		g := sim.New(telSvc, log)
		go g.Run(ctx, cfg.SimInterval)
	}

	go func() {
		log.Info("gRPC ingest listening", zap.String("addr", cfg.GRPCAddr))
		if err := grpcSrv.Serve(lis); err != nil {
			log.Error("gRPC serve", zap.Error(err))
		}
	}()

	go func() {
		log.Info("HTTP edge listening", zap.String("addr", cfg.HTTPAddr))
		if err := httpSrv.ListenAndServe(); !errors.Is(err, http.ErrServerClosed) {
			log.Error("HTTP serve", zap.Error(err))
		}
	}()

	quit := make(chan os.Signal, 1)
	signal.Notify(quit, syscall.SIGINT, syscall.SIGTERM)
	<-quit

	log.Info("shutting down")
	cancel()
	grpcSrv.GracefulStop()
	shutCtx, shutCancel := context.WithTimeout(context.Background(), shutdownTimeout)
	defer shutCancel()
	_ = httpSrv.Shutdown(shutCtx)
	return nil
}

// streamRecoveryInterceptor catches panics in streaming handlers and returns INTERNAL.
func streamRecoveryInterceptor(log *zap.Logger) grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) (err error) {
		defer func() {
			if r := recover(); r != nil {
				log.Error("gRPC stream panic", zap.Any("panic", r), zap.String("method", info.FullMethod))
				err = status.Errorf(codes.Internal, "internal server error")
			}
		}()
		return handler(srv, ss)
	}
}

// streamValidationInterceptor validates the first message of a client-stream via protovalidate.
// Per-message validation is done inside each streaming handler (interceptors run once per RPC).
func streamValidationInterceptor(v *protovalidate.Validator) grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) error {
		return handler(srv, &validatingStream{ServerStream: ss, v: v})
	}
}

type validatingStream struct {
	grpc.ServerStream
	v *protovalidate.Validator
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

// streamLoggingInterceptor logs each streaming RPC with method and outcome.
func streamLoggingInterceptor(log *zap.Logger) grpc.StreamServerInterceptor {
	return func(srv any, ss grpc.ServerStream, info *grpc.StreamServerInfo, handler grpc.StreamHandler) error {
		log.Info("gRPC stream started", zap.String("method", info.FullMethod))
		err := handler(srv, ss)
		if err != nil {
			log.Warn("gRPC stream error", zap.String("method", info.FullMethod), zap.Error(err))
		} else {
			log.Info("gRPC stream completed", zap.String("method", info.FullMethod))
		}
		return err
	}
}
