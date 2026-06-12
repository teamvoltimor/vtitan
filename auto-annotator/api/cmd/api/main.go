// Command api is the Go orchestration service for the auto-annotator: a Gin HTTP
// server over the SQLite manifest DB and the filesystem. ML compute (SAM,
// augmentation, training) stays in the Python workers and is reached over gRPC
// in later phases.
package main

import (
	"log/slog"
	"os"
	"strconv"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/http/handlers"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/store"
)

func main() {
	logger := slog.New(slog.NewJSONHandler(os.Stdout, nil))
	slog.SetDefault(logger)

	cfg, err := config.Load()
	if err != nil {
		slog.Error("load config", "error", err)
		os.Exit(1)
	}

	st, err := store.Open(cfg.DBPath)
	if err != nil {
		slog.Error("open store", "error", err, "db_path", cfg.DBPath)
		os.Exit(1)
	}
	defer st.Close()

	gin.SetMode(gin.ReleaseMode)
	app := handlers.New(cfg, st)
	router := app.Router()

	addr := ":" + strconv.Itoa(cfg.APIPort)
	slog.Info("starting api", "addr", addr, "db_path", cfg.DBPath, "data_dir", cfg.DataDir)
	if err := router.Run(addr); err != nil {
		slog.Error("server exited", "error", err)
		os.Exit(1)
	}
}
