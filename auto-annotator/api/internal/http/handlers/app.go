// Package handlers contains the Gin HTTP handlers and the router wiring. The App
// struct is the dependency-injection container, constructed once in main and
// shared (read-only) across handlers.
package handlers

import (
	"path/filepath"

	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/store"
)

// App holds the shared dependencies for all handlers.
type App struct {
	Cfg   config.Config
	Store *store.Store
}

// New builds the handler container.
func New(cfg config.Config, st *store.Store) *App {
	return &App{Cfg: cfg, Store: st}
}

// labelsDir returns the directory holding YOLO label files.
func (a *App) labelsDir() string {
	return filepath.Join(a.Cfg.DataDir, "labels")
}

// pendingDir returns the directory where uploaded images are staged.
func (a *App) pendingDir() string {
	return filepath.Join(a.Cfg.DataDir, "pending")
}
