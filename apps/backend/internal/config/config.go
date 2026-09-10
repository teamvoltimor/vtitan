package config

import (
	"time"

	"github.com/spf13/viper"
)

const (
	envPrefix            = "TELEMETRY"
	defaultHTTPAddr      = ":8010"
	defaultGRPCAddr      = ":9010"
	defaultHistorySize   = 360
	defaultSimIntervalMs = 2500
	defaultMaxSessions   = 20
	defaultSessionsDir   = "data/sessions"
	defaultDBPath        = "data/sessions.db"
	// defaultOpenAPISpecPath is relative to the process's working directory,
	// which matches how `task backend:dev`/`backend:run` invoke the server
	// (dir: backend). Override via TELEMETRY_OPENAPI_SPEC_PATH for any other
	// invocation (e.g. running the built binary from a different cwd).
	defaultOpenAPISpecPath = "../../contracts/openapi/openapi.yaml"
)

// Config holds all runtime configuration. String fields come first to minimize
// the GC-scanned pointer span.
type Config struct {
	HTTPAddr        string
	GRPCAddr        string
	SessionsDir     string
	DBPath          string
	OpenAPISpecPath string
	SimInterval     time.Duration
	HistorySize     int
	MaxSessions     int
	Dev             bool
}

// Load reads configuration from TELEMETRY_* environment variables and returns
// a Config populated with defaults for any value that is not set.
func Load() *Config {
	v := viper.New()
	v.SetEnvPrefix(envPrefix)
	v.AutomaticEnv()

	v.SetDefault("http_addr", defaultHTTPAddr)
	v.SetDefault("grpc_addr", defaultGRPCAddr)
	v.SetDefault("history_size", defaultHistorySize)
	v.SetDefault("dev", false)
	v.SetDefault("sim_interval_ms", defaultSimIntervalMs)
	v.SetDefault("max_sessions", defaultMaxSessions)
	v.SetDefault("sessions_dir", defaultSessionsDir)
	v.SetDefault("db_path", defaultDBPath)
	v.SetDefault("openapi_spec_path", defaultOpenAPISpecPath)

	return &Config{
		HTTPAddr:        v.GetString("http_addr"),
		GRPCAddr:        v.GetString("grpc_addr"),
		HistorySize:     v.GetInt("history_size"),
		Dev:             v.GetBool("dev"),
		SimInterval:     time.Duration(v.GetInt("sim_interval_ms")) * time.Millisecond,
		MaxSessions:     v.GetInt("max_sessions"),
		SessionsDir:     v.GetString("sessions_dir"),
		DBPath:          v.GetString("db_path"),
		OpenAPISpecPath: v.GetString("openapi_spec_path"),
	}
}
