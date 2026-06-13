// Command api is the Go orchestration service for the auto-annotator: a Gin HTTP
// server over the SQLite manifest DB and the filesystem. ML compute (SAM,
// augmentation, training) stays in the Python workers and is reached over gRPC
// in later phases.
package main

import (
	"fmt"
	"log/slog"
	"os"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/compute"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/handlers"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/jobs"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store"
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

	computeClients, err := compute.NewGRPC(cfg.SegmentAddr, cfg.AugmentAddr, cfg.TrainAddr)
	if err != nil {
		return fmt.Errorf("init compute clients: %w", err)
	}
	defer computeClients.Close()

	gin.SetMode(gin.ReleaseMode)
	app := handlers.New(cfg, st, jobs.New(), computeClients)
	router := app.Router()

	addr := ":" + strconv.Itoa(cfg.APIPort)
	slog.Info("starting api", "addr", addr, "db_path", cfg.DBPath, "data_dir", cfg.DataDir)
	if err := router.Run(addr); err != nil {
		return fmt.Errorf("server: %w", err)
	}
	return nil
}
