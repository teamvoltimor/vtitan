package config

const (
	// EnvAPIPort is the API server port environment variable key.
	EnvAPIPort = "API_PORT"
	// EnvDBPath is the database path environment variable key.
	EnvDBPath = "DB_PATH"
	// EnvDataDir is the data directory environment variable key.
	EnvDataDir = "DATA_DIR"
	// EnvModelsConfig is the models config path environment variable key.
	EnvModelsConfig = "MODELS_CONFIG"
	// EnvAPIPublicURL is the API public URL environment variable key.
	EnvAPIPublicURL = "API_PUBLIC_URL"
	// EnvCORSOrigins is the CORS origins environment variable key.
	EnvCORSOrigins = "CORS_ORIGINS"
	// EnvSegmentAddr is the segmentation gRPC address environment variable key.
	EnvSegmentAddr = "SEGMENT_GRPC_ADDR"
	// EnvAugmentAddr is the augmentation gRPC address environment variable key.
	EnvAugmentAddr = "AUGMENT_GRPC_ADDR"
	// EnvTrainAddr is the training gRPC address environment variable key.
	EnvTrainAddr = "TRAIN_GRPC_ADDR"
	// EnvOpenAPIPath is the OpenAPI spec file path environment variable key.
	EnvOpenAPIPath = "OPENAPI_SPEC_PATH"

	// DefaultAPIPort is the default API server port.
	DefaultAPIPort = 8000
	// DefaultDBPath is the default database path.
	DefaultDBPath = "./data/manifest.db"
	// DefaultDataDir is the default data directory.
	DefaultDataDir = "./data"
	// DefaultModelsConfig is the default models config path.
	DefaultModelsConfig = "./config/models.toml"
	// DefaultCORSOrigins is the default CORS origins (localhost for dev).
	DefaultCORSOrigins = "http://localhost:5173,http://localhost:3000"
	// DefaultOpenAPIPath is the default path to the OpenAPI spec file.
	DefaultOpenAPIPath = "api/openapi.yaml"

	// ConfigFileName is the configuration file name.
	ConfigFileName = ".env"
	// ConfigFileType is the configuration file type.
	ConfigFileType = "env"
)
