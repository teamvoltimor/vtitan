// Package config loads typed application configuration from environment
// variables and an optional .env file via Viper.
package config

import (
	"fmt"
	"strings"

	"github.com/spf13/viper"
)

// Config is the typed application configuration.
type Config struct {
	APIPort      int
	DBPath       string
	DataDir      string
	ModelsConfig string
	APIPublicURL string
	CORSOrigins  []string

	// Compute-worker gRPC addresses — consumed in later phases (segmentation,
	// augmentation, training). Defined now so the contract is visible.
	SegmentAddr string
	AugmentAddr string
	TrainAddr   string
}

// Load reads configuration from the environment (and an optional .env file),
// applying defaults that mirror the Python backend.
func Load() (Config, error) {
	v := viper.New()
	v.SetConfigName(".env")
	v.SetConfigType("env")
	v.AddConfigPath(".")
	// .env is optional; ignore a missing-file error but surface a malformed one.
	if err := v.ReadInConfig(); err != nil {
		if _, ok := err.(viper.ConfigFileNotFoundError); !ok {
			return Config{}, fmt.Errorf("read config: %w", err)
		}
	}
	v.AutomaticEnv()

	v.SetDefault("API_PORT", 8000)
	v.SetDefault("DB_PATH", "./data/manifest.db")
	v.SetDefault("DATA_DIR", "./data")
	v.SetDefault("MODELS_CONFIG", "./config/models.toml")
	v.SetDefault("API_PUBLIC_URL", "")
	v.SetDefault("CORS_ORIGINS", "http://localhost:5173,http://localhost:3000")
	v.SetDefault("SEGMENT_GRPC_ADDR", "")
	v.SetDefault("AUGMENT_GRPC_ADDR", "")
	v.SetDefault("TRAIN_GRPC_ADDR", "")

	port := v.GetInt("API_PORT")
	publicURL := v.GetString("API_PUBLIC_URL")
	if publicURL == "" {
		publicURL = fmt.Sprintf("http://localhost:%d", port)
	}

	return Config{
		APIPort:      port,
		DBPath:       v.GetString("DB_PATH"),
		DataDir:      v.GetString("DATA_DIR"),
		ModelsConfig: v.GetString("MODELS_CONFIG"),
		APIPublicURL: publicURL,
		CORSOrigins:  splitAndTrim(v.GetString("CORS_ORIGINS")),
		SegmentAddr:  v.GetString("SEGMENT_GRPC_ADDR"),
		AugmentAddr:  v.GetString("AUGMENT_GRPC_ADDR"),
		TrainAddr:    v.GetString("TRAIN_GRPC_ADDR"),
	}, nil
}

func splitAndTrim(csv string) []string {
	out := make([]string, 0)
	for _, part := range strings.Split(csv, ",") {
		if p := strings.TrimSpace(part); p != "" {
			out = append(out, p)
		}
	}
	return out
}
