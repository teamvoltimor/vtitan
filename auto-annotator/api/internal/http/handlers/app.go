// Package handlers contains the Gin HTTP handlers and the router wiring. The App
// struct is the dependency-injection container, constructed once in main and
// shared (read-only) across handlers.
package handlers

import (
	"path/filepath"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/compute"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/jobs"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store"
)

// App holds the shared dependencies for all handlers.
type App struct {
	Compute compute.Clients
	Store   *store.Store
	Jobs    *jobs.Manager
	Cfg     config.Config
}

// New builds the handler container.
func New(cfg config.Config, st *store.Store, jm *jobs.Manager, cc compute.Clients) *App {
	return &App{Cfg: cfg, Store: st, Jobs: jm, Compute: cc}
}

// labelsDir returns the directory holding YOLO label files.
func (a *App) labelsDir() string {
	return filepath.Join(a.Cfg.DataDir, LabelsDirName)
}

// imagesDir returns the directory where annotated images are organized by class.
func (a *App) imagesDir() string {
	return filepath.Join(a.Cfg.DataDir, ImagesDirName)
}

// pendingDir returns the directory where uploaded images are staged.
func (a *App) pendingDir() string {
	return filepath.Join(a.Cfg.DataDir, PendingDirName)
}

// dataYamlPath returns the path to the generated YOLO data.yaml.
func (a *App) dataYamlPath() string {
	return filepath.Join(a.Cfg.DataDir, DataYAMLName)
}
