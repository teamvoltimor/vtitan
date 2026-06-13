// Package config loads typed application configuration from environment variables
// and an optional .env file via Viper. It also holds typed configuration and constants.
package config

import (
	"errors"
	"fmt"
	"strings"

	"github.com/spf13/viper"
)

// Config is the typed application configuration.
type Config struct {
	DBPath       string
	DataDir      string
	ModelsConfig string
	APIPublicURL string
	SegmentAddr  string
	AugmentAddr  string
	TrainAddr    string
	CORSOrigins  []string
	APIPort      int
}

// Load reads configuration from the environment (and an optional .env file),
// applying defaults that mirror the Python backend.
func Load() (Config, error) {
	v := viper.New()
	v.SetConfigName(ConfigFileName)
	v.SetConfigType(ConfigFileType)
	v.AddConfigPath(".")
	// .env is optional; ignore a missing-file error but surface a malformed one.
	if err := v.ReadInConfig(); err != nil {
		var notFound viper.ConfigFileNotFoundError
		if !errors.As(err, &notFound) {
			return Config{}, fmt.Errorf("read config: %w", err)
		}
	}
	v.AutomaticEnv()

	v.SetDefault(EnvAPIPort, DefaultAPIPort)
	v.SetDefault(EnvDBPath, DefaultDBPath)
	v.SetDefault(EnvDataDir, DefaultDataDir)
	v.SetDefault(EnvModelsConfig, DefaultModelsConfig)
	v.SetDefault(EnvAPIPublicURL, "")
	v.SetDefault(EnvCORSOrigins, DefaultCORSOrigins)
	v.SetDefault(EnvSegmentAddr, "")
	v.SetDefault(EnvAugmentAddr, "")
	v.SetDefault(EnvTrainAddr, "")

	port := v.GetInt(EnvAPIPort)
	publicURL := v.GetString(EnvAPIPublicURL)
	if publicURL == "" {
		publicURL = fmt.Sprintf("http://localhost:%d", port)
	}

	return Config{
		APIPort:      port,
		DBPath:       v.GetString(EnvDBPath),
		DataDir:      v.GetString(EnvDataDir),
		ModelsConfig: v.GetString(EnvModelsConfig),
		APIPublicURL: publicURL,
		CORSOrigins:  splitAndTrim(v.GetString(EnvCORSOrigins)),
		SegmentAddr:  v.GetString(EnvSegmentAddr),
		AugmentAddr:  v.GetString(EnvAugmentAddr),
		TrainAddr:    v.GetString(EnvTrainAddr),
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
