package handlers

import (
	"net/http"

	"github.com/BurntSushi/toml"
	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
)

// modelsConfig mirrors the relevant fields of config/models.toml.
type modelsConfig struct {
	DefaultModel string `toml:"default_model"`
	Models       []struct {
		ID           string `toml:"id"`
		Label        string `toml:"label"`
		Type         string `toml:"type"`
		SupportsText bool   `toml:"supports_text"`
	} `toml:"models"`
}

// ListModels returns the configured models from models.toml. GET /models
//
// available/active are resolved against the SAM compute worker in a later phase;
// for now every configured model is reported available and the default is active.
func (a *App) ListModels(c *gin.Context) {
	var cfg modelsConfig
	if _, err := toml.DecodeFile(a.Cfg.ModelsConfig, &cfg); err != nil {
		// Missing/unreadable config yields an empty list, matching the Python handler.
		c.JSON(http.StatusOK, []dto.ModelItem{})
		return
	}

	items := make([]dto.ModelItem, 0, len(cfg.Models))
	for _, m := range cfg.Models {
		label := m.Label
		if label == "" {
			label = m.ID
		}
		items = append(items, dto.ModelItem{
			ID:           m.ID,
			Label:        label,
			ModelType:    m.Type,
			Available:    true,
			Active:       m.ID == cfg.DefaultModel,
			SupportsText: m.SupportsText,
		})
	}
	c.JSON(http.StatusOK, items)
}
