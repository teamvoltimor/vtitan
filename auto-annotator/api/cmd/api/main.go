// Command api is the Go orchestration service for the auto-annotator: a Gin HTTP
// server over the SQLite manifest DB and the filesystem. ML compute (SAM,
// augmentation, training) stays in the Python workers and is reached over gRPC.
package main

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"net/http"
	"os"
	"os/signal"
	"strconv"
	"syscall"
	"time"

	"github.com/gin-gonic/gin"

	annotationdomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	annotsqlite "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation/sqlite"
	computedomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute"
	computegrpc "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute/grpc"
	computesqlite "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/compute/sqlite"
	gallerydomain "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/gallery"
	gallerysqlite "github.com/teamvoltimor/vtitan/auto-annotator/api/domain/gallery/sqlite"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/job"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/store"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/config"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/http/handlers"
)

const readHeaderTimeout = 5 * time.Second

// shutdownGracePeriod bounds how long we wait for in-flight requests (e.g. a
// SaveAnnotations or ImportGallery mid-write) to finish once a shutdown
// signal arrives, before forcing the listener closed.
const shutdownGracePeriod = 10 * time.Second

func main() {
	slog.SetDefault(slog.New(slog.NewJSONHandler(os.Stdout, nil)))
	if err := run(); err != nil {
		slog.Error("startup failed", "error", err)
		os.Exit(1)
	}
}

func run() error {
	cfg, err := config.Load()
	if err != nil {
		return fmt.Errorf("load config: %w", err)
	}

	st, err := store.Open(cfg.DBPath)
	if err != nil {
		return fmt.Errorf("open store: %w", err)
	}
	defer st.Close()

	clients, err := computegrpc.NewGRPC(cfg.SegmentAddr, cfg.AugmentAddr, cfg.TrainAddr)
	if err != nil {
		return fmt.Errorf("init compute clients: %w", err)
	}
	defer clients.Close()

	q := st.Q
	annSvc := annotationdomain.NewService(annotsqlite.New(q), cfg)
	galSvc := gallerydomain.NewService(gallerysqlite.New(q), cfg)
	compSvc := computedomain.NewService(clients, job.New(), computesqlite.New(q), cfg)

	gin.SetMode(gin.ReleaseMode)
	app := handlers.New(cfg, annSvc, galSvc, compSvc, st.DB)
	router := app.Router()

	addr := ":" + strconv.Itoa(cfg.APIPort)
	srv := &http.Server{
		Addr:              addr,
		Handler:           router,
		ReadHeaderTimeout: readHeaderTimeout,
	}

	serveErr := make(chan error, 1)
	go func() {
		slog.Info("starting api", "addr", addr, "db_path", cfg.DBPath, "data_dir", cfg.DataDir)
		if err := srv.ListenAndServe(); err != nil && !errors.Is(err, http.ErrServerClosed) {
			serveErr <- fmt.Errorf("server: %w", err)
			return
		}
		serveErr <- nil
	}()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	select {
	case err := <-serveErr:
		return err
	case <-ctx.Done():
	}

	slog.Info("shutdown signal received, draining connections")
	shutdownCtx, cancel := context.WithTimeout(context.Background(), shutdownGracePeriod)
	defer cancel()
	if err := srv.Shutdown(shutdownCtx); err != nil {
		return fmt.Errorf("graceful shutdown: %w", err)
	}
	return <-serveErr
}
