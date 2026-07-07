// Command api is the Go orchestration service for the auto-annotator: a Gin HTTP
// server over the SQLite manifest DB and the filesystem. ML compute (SAM,
// augmentation, training) stays in the Python workers and is reached over gRPC.
package main

import (
	"fmt"
	"log/slog"
	"os"
	"strconv"

	"github.com/gin-gonic/gin"

	annotationdomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation"
	annotsqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation/sqlite"
	computedomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute"
	computegrpc "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute/grpc"
	computesqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute/sqlite"
	gallerydomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/gallery"
	gallerysqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/gallery/sqlite"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/job"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/handlers"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/store"
)

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
	slog.Info("starting api", "addr", addr, "db_path", cfg.DBPath, "data_dir", cfg.DataDir)
	if err := router.Run(addr); err != nil {
		return fmt.Errorf("server: %w", err)
	}
	return nil
}
