package vtcli

// The container, contract and pipeline trees of the two specialist apps.
// They are long Docker-bound workflows rather than daily verbs, but the
// picker shows the whole surface, so they are wrapped here (ADR 0096); the
// anti-drift checks keep a task from being silently dropped from both the
// spec and the exclusion lists.

// autoAnnotatorToolsSpec wraps the auto-annotator contract, database and
// container workflows.
var autoAnnotatorToolsSpec = []Command{
	{
		Path:  []string{"annotator", "proto", "gen"},
		Task:  "auto-annotator:proto:gen",
		Group: groupRun,
		Short: "Regenerate the Go + Python gRPC stubs from the proto contract",
	},
	{
		Path:  []string{"annotator", "proto", "gen", "go"},
		Task:  "auto-annotator:proto:gen:go",
		Group: groupRun,
		Short: "Regenerate the Go gRPC stubs (buf)",
	},
	{
		Path:  []string{"annotator", "proto", "gen", "py"},
		Task:  "auto-annotator:proto:gen:py",
		Group: groupRun,
		Short: "Regenerate the Python gRPC stubs and .pyi type stubs",
	},
	{
		Path:  []string{"annotator", "proto", "lint"},
		Task:  "auto-annotator:proto:lint",
		Group: groupCheck,
		Short: "Lint the shared proto contract (buf lint)",
	},
	{
		Path:  []string{"annotator", "proto", "breaking"},
		Task:  "auto-annotator:proto:breaking",
		Group: groupCheck,
		Short: "Check the proto contract for breaking changes",
	},
	{
		Path:  []string{"annotator", "openapi", "validate"},
		Task:  "auto-annotator:openapi:validate",
		Group: groupCheck,
		Short: "Lint the OpenAPI spec with Redocly",
	},
	{
		Path:  []string{"annotator", "openapi", "generate"},
		Task:  "auto-annotator:openapi:generate:all",
		Group: groupRun,
		Short: "Generate the API types for backend and frontend",
	},
	{
		Path:  []string{"annotator", "openapi", "generate", "backend"},
		Task:  "auto-annotator:openapi:generate:backend",
		Group: groupRun,
		Short: "Generate the Go types from the OpenAPI spec",
	},
	{
		Path:  []string{"annotator", "openapi", "generate", "frontend"},
		Task:  "auto-annotator:openapi:generate:frontend",
		Group: groupRun,
		Short: "Generate the TypeScript types from the OpenAPI spec",
	},
	{
		Path:  []string{"annotator", "db", "sqlc"},
		Task:  "auto-annotator:sqlc:gen",
		Group: groupRun,
		Short: "Regenerate the Go DB access code from the migrations (sqlc)",
	},
	{
		Path:  []string{"annotator", "db", "migrate", "new"},
		Task:  "auto-annotator:db:migrate:new",
		Group: groupSetup,
		Flags: []Flag{{Name: "name", Var: "NAME", Required: true, Usage: "migration name"}},
		Short: "Create a new goose migration under db/migrations",
	},
	{
		Path:  []string{"annotator", "db", "migrate", "status"},
		Task:  "auto-annotator:db:migrate:status",
		Group: groupCheck,
		Flags: []Flag{{Name: "db-path", Var: "DB_PATH", Usage: "SQLite database path"}},
		Short: "Show which migrations are applied or pending",
	},
	{
		Path:  []string{"annotator", "docker", "build", "api"},
		Task:  "auto-annotator:docker:build:api",
		Group: groupRun,
		Short: "Build the Go API Docker image",
	},
	{
		Path:  []string{"annotator", "docker", "build", "ml"},
		Task:  "auto-annotator:docker:build:ml-service",
		Group: groupRun,
		Short: "Build the ML service Docker image",
	},
	{
		Path:  []string{"annotator", "docker", "up"},
		Task:  "auto-annotator:docker:up",
		Group: groupRun,
		Flags: []Flag{
			{Name: "detached", Var: "DETACHED", Kind: FlagBool, Default: "false", Usage: "run in the background"},
		},
		Short: "Start the compose stack (ml-service + api + frontend)",
	},
	{
		Path:  []string{"annotator", "docker", "down"},
		Task:  "auto-annotator:docker:down",
		Group: groupClean,
		Short: "Stop and remove the compose services",
	},
	{
		Path:  []string{"annotator", "docker", "logs"},
		Task:  "auto-annotator:docker:logs",
		Group: groupRun,
		Flags: []Flag{{Name: "service", Var: "SERVICE", Usage: "api|ml-service|frontend (default all)"}},
		Short: "Follow the compose logs",
	},
	{
		Path:  []string{"annotator", "docker", "run", "api"},
		Task:  "auto-annotator:docker:run:api",
		Group: groupRun,
		Short: "Run the Go API container",
	},
	{
		Path:  []string{"annotator", "docker", "run", "ml"},
		Task:  "auto-annotator:docker:run:ml-service",
		Group: groupRun,
		Short: "Run the ML service container with GPU and volume mounts",
	},
	{
		Path:  []string{"annotator", "docker", "stack"},
		Task:  "auto-annotator:dev:docker",
		Group: groupRun,
		Flags: []Flag{
			{Name: "detached", Var: "DETACHED", Kind: FlagBool, Default: "false", Usage: "run in the background"},
		},
		Short: "Start the full Docker stack",
	},
	{
		Path:  []string{"annotator", "frontend", "preview"},
		Task:  "auto-annotator:frontend:preview",
		Group: groupRun,
		Short: "Preview the production build locally",
	},
}

// hailoToolsSpec wraps the Hailo Docker suite, the ONNX to HEF pipeline and
// the GMR and accuracy sweeps.
var hailoToolsSpec = []Command{
	{
		Path:  []string{"hailo", "docker", "dry"},
		Task:  "hailo:docker:dry",
		Group: groupRun,
		Short: "Print the docker run command without executing it",
	},
	{
		Path:  []string{"hailo", "docker", "load"},
		Task:  "hailo:docker:load",
		Group: groupRun,
		Short: "Load the Hailo AI Software Suite image from its tarball",
	},
	{
		Path:  []string{"hailo", "docker", "logs"},
		Task:  "hailo:docker:logs",
		Group: groupRun,
		Short: "Follow the container logs",
	},
	{
		Path:  []string{"hailo", "docker", "run"},
		Task:  "hailo:docker:run",
		Group: groupRun,
		Short: "Start the Hailo AI Software Suite container",
	},
	{
		Path:  []string{"hailo", "docker", "run-compile-only"},
		Task:  "hailo:docker:run-compile-only",
		Group: groupRun,
		Short: "Start a minimal detached container (Docker Desktop)",
	},
	{
		Path:  []string{"hailo", "docker", "status"},
		Task:  "hailo:docker:status",
		Group: groupCheck,
		Short: "Check whether the container is running",
	},
	{
		Path:  []string{"hailo", "docker", "stop"},
		Task:  "hailo:docker:stop",
		Group: groupClean,
		Short: "Stop the running Hailo container",
	},
	{
		Path:  []string{"hailo", "compile", "dry"},
		Task:  "hailo:compile:dry",
		Group: groupRun,
		Short: "Print the ONNX to HEF compile command",
	},
	{
		Path:  []string{"hailo", "compile", "run"},
		Task:  "hailo:compile:run",
		Group: groupRun,
		Short: "Compile ONNX to Hailo HEF in Docker",
	},
	{
		Path:  []string{"hailo", "profile", "dry"},
		Task:  "hailo:profile:dry",
		Group: groupRun,
		Short: "Print the HEF profiling command",
	},
	{
		Path:  []string{"hailo", "profile", "run"},
		Task:  "hailo:profile:run",
		Group: groupRun,
		Short: "Profile a HEF model for performance",
	},
	{
		Path:  []string{"hailo", "eval", "dry"},
		Task:  "hailo:eval:dry",
		Group: groupRun,
		Short: "Print the evaluation command",
	},
	{
		Path:  []string{"hailo", "eval", "run"},
		Task:  "hailo:eval:run",
		Group: groupRun,
		Short: "Evaluate a model on the target",
	},
	{
		Path:  []string{"hailo", "eval", "visual"},
		Task:  "hailo:eval:visual",
		Group: groupRun,
		Short: "Evaluate with visualization",
	},
	{
		Path:  []string{"hailo", "test", "onnx"},
		Task:  "hailo:test:onnx",
		Group: groupRun,
		Short: "Run inference on an ONNX model",
	},
	{
		Path:  []string{"hailo", "test", "run"},
		Task:  "hailo:test:run",
		Group: groupRun,
		Flags: []Flag{{Name: "backend", Var: "BACKEND", Required: true, Usage: "pt|onnx|ultraonnx"}},
		Short: "Run inference on images",
	},
	{
		Path:  []string{"hailo", "test", "ultraonnx"},
		Task:  "hailo:test:ultraonnx",
		Group: groupRun,
		Short: "Run inference with the Ultralytics ONNX backend",
	},
	{
		Path:  []string{"hailo", "model", "export-inspect"},
		Task:  "hailo:model:export-inspect",
		Group: groupRun,
		Short: "Export then inspect the resulting ONNX model",
	},
	{
		Path:  []string{"hailo", "calib", "convert"},
		Task:  "hailo:calib:convert",
		Group: groupRun,
		Short: "Convert calibration images to float32 NPY",
	},
	{
		Path:  []string{"hailo", "calib", "download"},
		Task:  "hailo:calib:download",
		Group: groupRun,
		Short: "Download the COCO 2017 calibration images",
	},
	{
		Path:  []string{"hailo", "calib", "prepare"},
		Task:  "hailo:calib:prepare",
		Group: groupRun,
		Short: "Download and convert the calibration data",
	},
	{
		Path:  []string{"hailo", "stage", "no-calib"},
		Task:  "hailo:stage:no-calib",
		Group: groupRun,
		Short: "Stage the model ONNX without calibration data",
	},
	{
		Path:  []string{"hailo", "stage", "run"},
		Task:  "hailo:stage:run",
		Group: groupRun,
		Short: "Stage the model ONNX and calibration data for Docker",
	},
	{
		Path:  []string{"hailo", "gmr", "export"},
		Task:  "hailo:gmr:export",
		Group: groupRun,
		Short: "Export the retrained GMR checkpoint to ONNX",
	},
	{
		Path:  []string{"hailo", "gmr", "stage"},
		Task:  "hailo:gmr:stage",
		Group: groupRun,
		Short: "Stage the GMR ONNX and prism calibration images",
	},
	{
		Path:  []string{"hailo", "gmr", "stage-eval"},
		Task:  "hailo:gmr:stage-eval",
		Group: groupRun,
		Short: "Stage the GMR images and labels for the evaluators",
	},
	{
		Path:  []string{"hailo", "gmr", "compile"},
		Task:  "hailo:gmr:compile",
		Group: groupRun,
		Short: "Compile the GMR ONNX to Hailo HEF",
	},
	{
		Path:  []string{"hailo", "gmr", "compile-dry"},
		Task:  "hailo:gmr:compile-dry",
		Group: groupRun,
		Short: "Print the GMR compile command",
	},
	{
		Path:  []string{"hailo", "gmr", "compile-performance"},
		Task:  "hailo:gmr:compile-performance",
		Group: groupRun,
		Short: "Compile GMR at the highest optimization level",
	},
	{
		Path:  []string{"hailo", "gmr", "workflow"},
		Task:  "hailo:gmr:workflow",
		Group: groupRun,
		Short: "GMR pipeline: export, stage, compile",
	},
	{
		Path:  []string{"hailo", "accuracy", "all"},
		Task:  "hailo:accuracy:all",
		Group: groupRun,
		Short: "Full accuracy comparison: float ceiling, then every HAR",
	},
	{
		Path:  []string{"hailo", "accuracy", "compare"},
		Task:  "hailo:accuracy:compare",
		Group: groupRun,
		Short: "Score the compiled HARs against each other",
	},
	{
		Path:  []string{"hailo", "accuracy", "float"},
		Task:  "hailo:accuracy:float",
		Group: groupRun,
		Short: "Score the float checkpoint as the accuracy ceiling",
	},
	{
		Path:  []string{"hailo", "accuracy", "sync"},
		Task:  "hailo:accuracy:sync",
		Group: groupRun,
		Short: "Copy the evaluators into the shared mount",
	},
	{
		Path:  []string{"hailo", "accuracy", "sync-validate"},
		Task:  "hailo:accuracy:sync-validate",
		Group: groupCheck,
		Short: "Check for drift between eval/ and shared_with_docker/eval/",
	},
	{
		Path:  []string{"hailo", "workflow", "compile"},
		Task:  "hailo:workflow:compile",
		Group: groupRun,
		Short: "Compile workflow: stage, then compile in Docker",
	},
	{
		Path:  []string{"hailo", "workflow", "eval"},
		Task:  "hailo:workflow:eval",
		Group: groupRun,
		Short: "Evaluation workflow: compile, eval, profile",
	},
	{
		Path:  []string{"hailo", "workflow", "full"},
		Task:  "hailo:workflow:full",
		Group: groupRun,
		Short: "Local pipeline: export, calib, test, stage, start Docker",
	},
	{
		Path:  []string{"hailo", "log", "run"},
		Task:  "hailo:log:run",
		Group: groupRun,
		Flags: []Flag{
			{Name: "level", Var: "LEVEL", Usage: "DEBUG|INFO|WARNING"},
			{Name: "args", Var: "ARGS", Usage: "the command to run"},
		},
		Short: "Run a command with a specific log level",
	},
}
