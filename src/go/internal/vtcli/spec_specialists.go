package vtcli

// Two specialist toolchains, each a self-contained app with its own frontend,
// service and build. Both are included WITHOUT flatten, so their task names
// carry the include key as a prefix (auto-annotator:*, hailo:*), which is why
// they nest one level deeper than the flattened domains.
//
// Only the daily verbs are wrapped. The rest -- hailo's docker / compile /
// calib / gmr / accuracy trees and auto-annotator's db / proto / openapi /
// docker trees -- are long container-bound workflows with their own READMEs,
// and every one of them is still reachable through `vt task`. They are
// excluded by name below so the anti-drift check keeps demanding a decision
// for anything new.

// autoAnnotatorSpec wraps the daily auto-annotator tasks
// (other/apps/auto-annotator/Taskfile.yml). The whole-stack verbs come first;
// each of the three services then has its own namespace, because that is how
// the app is split and how its Taskfile names them.
var autoAnnotatorSpec = []Command{
	{
		Path:      []string{"annotator", "dev"},
		Task:      "auto-annotator:dev:serve",
		Group:     groupRun,
		Short:     "Run ml-service + api + frontend concurrently",
		Heavy:     true,
		Platforms: []Platform{platformLinux, platformDarwin},
		Variants: []Variant{{
			Flag: "install", Task: "auto-annotator:dev:local", Usage: "install the dependencies first",
		}},
	},
	{
		Path:  []string{"annotator", "lint"},
		Task:  "auto-annotator:lint:all",
		Group: groupCheck,
		Short: "Lint ml-service, frontend and the Go API",
	},
	{
		Path:  []string{"annotator", "format"},
		Task:  "auto-annotator:format:all",
		Group: groupCheck,
		Short: "Format ml-service, frontend, the Go API and the proto",
	},
	{
		Path:  []string{"annotator", "install"},
		Task:  "auto-annotator:dev:init",
		Group: groupSetup,
		Short: "Install the ML service and frontend dependencies",
	},
	{
		Path:      []string{"annotator", "clean"},
		Task:      "auto-annotator:clean:build",
		Group:     groupClean,
		Short:     "Remove build artifacts and caches",
		Platforms: []Platform{platformLinux, platformDarwin},
		Variants: []Variant{{
			Flag: "deep", Task: "auto-annotator:clean:deep", Usage: "the dependencies too",
		}},
	},
	{
		Path:  []string{"annotator", "ml", "dev"},
		Task:  "auto-annotator:ml-service:dev",
		Group: groupRun,
		Short: "Start the SAM model server + gRPC compute service",
		Heavy: true,
		Flags: []Flag{{Name: "model-id", Var: "MODEL_ID", Usage: "pre-load a specific model"}},
	},
	{
		Path:  []string{"annotator", "ml", "test"},
		Task:  "auto-annotator:ml-service:test",
		Group: groupCheck,
		Short: "ML service unit tests (no GPU or weights)",
	},
	{
		Path:  []string{"annotator", "ml", "typecheck"},
		Task:  "auto-annotator:ml-service:typecheck",
		Group: groupCheck,
		Short: "mypy on the ML service",
	},
	{
		Path:  []string{"annotator", "ml", "install"},
		Task:  "auto-annotator:ml-service:sync",
		Group: groupSetup,
		Short: "Install the ML service dependencies (uv)",
	},
	{
		Path:  []string{"annotator", "api", "dev"},
		Task:  "auto-annotator:api:dev",
		Group: groupRun,
		Short: "Run the Go orchestration API (Gin + SQLite + gRPC)",
		Heavy: true,
		Flags: []Flag{{Name: "model-id", Var: "MODEL_ID", Usage: "pre-load a specific model"}},
	},
	{
		Path:  []string{"annotator", "api", "build"},
		Task:  "auto-annotator:api:build",
		Group: groupRun,
		Short: "Build the Go API binary",
	},
	{
		Path:  []string{"annotator", "api", "test"},
		Task:  "auto-annotator:api:test",
		Group: groupCheck,
		Short: "Run the Go API tests",
	},
	{
		Path:  []string{"annotator", "api", "typecheck"},
		Task:  "auto-annotator:api:typecheck",
		Group: groupCheck,
		Short: "go vet the Go API",
	},
	{
		Path:     []string{"annotator", "api", "align"},
		Task:     "auto-annotator:align:api",
		Group:    groupCheck,
		Short:    "Check Go struct alignment (betteralign)",
		Variants: []Variant{fixVariant("auto-annotator:align:fix:api")},
	},
	{
		Path:  []string{"annotator", "api", "install"},
		Task:  "auto-annotator:api:deps",
		Group: groupSetup,
		Short: "Download the Go API dependencies",
	},
	{
		Path:  []string{"annotator", "frontend", "dev"},
		Task:  "auto-annotator:frontend:dev",
		Group: groupRun,
		Short: "Vite dev server on :5173",
		Heavy: true,
	},
	{
		Path:  []string{"annotator", "frontend", "build"},
		Task:  "auto-annotator:frontend:build",
		Group: groupRun,
		Short: "Build the frontend for production",
	},
	{
		Path:  []string{"annotator", "frontend", "test"},
		Task:  "auto-annotator:frontend:test",
		Group: groupCheck,
		Short: "Frontend parity tests (Vitest)",
	},
	{
		Path:  []string{"annotator", "frontend", "install"},
		Task:  "auto-annotator:frontend:install",
		Group: groupSetup,
		Short: "Install the frontend dependencies",
	},
}

// autoAnnotatorExclusions are the auto-annotator tasks left out of the tree:
// the container, database, contract and docker workflows.
var autoAnnotatorExclusions = map[string]string{
	"auto-annotator:default":                   "Task's own task list; `vt task` replaces it",
	"auto-annotator:align":                     "ambiguous alias of align:api; use annotator api align",
	"auto-annotator:proto:gen":                 "proto contract workflow; use vt task",
	"auto-annotator:proto:gen:go":              "proto contract workflow; use vt task",
	"auto-annotator:proto:gen:py":              "proto contract workflow; use vt task",
	"auto-annotator:proto:lint":                "proto contract workflow; use vt task",
	"auto-annotator:proto:breaking":            "proto contract workflow; use vt task",
	"auto-annotator:openapi:validate":          "OpenAPI workflow; use vt task",
	"auto-annotator:openapi:generate:all":      "OpenAPI workflow; use vt task",
	"auto-annotator:openapi:generate:backend":  "OpenAPI workflow; use vt task",
	"auto-annotator:openapi:generate:frontend": "OpenAPI workflow; use vt task",
	"auto-annotator:sqlc:gen":                  "database codegen; use vt task",
	"auto-annotator:db:migrate:new":            "database migration; use vt task",
	"auto-annotator:db:migrate:status":         "database migration; use vt task",
	"auto-annotator:docker:build:api":          "docker image build; use vt task",
	"auto-annotator:docker:build:ml-service":   "docker image build; use vt task",
	"auto-annotator:docker:down":               "docker compose; use vt task (vt docker down)",
	"auto-annotator:docker:logs":               "docker compose; use vt task (vt docker logs)",
	"auto-annotator:docker:run:api":            "docker container; use vt task",
	"auto-annotator:docker:run:ml-service":     "docker container; use vt task",
	"auto-annotator:docker:up":                 "docker compose; use vt task (vt docker up)",
	"auto-annotator:dev:docker":                "docker compose stack; use vt task",
	"auto-annotator:lint:api":                  "covered by annotator lint; use vt task for api only",
	"auto-annotator:lint:frontend":             "covered by annotator lint; use vt task for frontend only",
	"auto-annotator:lint:ml-service":           "covered by annotator lint; use vt task for ml-service only",
	"auto-annotator:format:api":                "covered by annotator format; use vt task for api only",
	"auto-annotator:format:frontend":           "covered by annotator format; use vt task for frontend only",
	"auto-annotator:format:ml-service":         "covered by annotator format; use vt task for ml-service only",
	"auto-annotator:format:proto":              "covered by annotator format; use vt task for the proto only",
	"auto-annotator:frontend:preview":          "previews the production build; use vt task",
}

// hailoSpec wraps the daily Hailo/ML tasks (other/ml/hailo/Taskfile.yml).
// Single-task namespaces are flattened (`hailo test`, not `hailo test unit`)
// and the three cleans are one command with two narrowing switches.
var hailoSpec = []Command{
	{
		Path:     []string{"hailo", "model", "export"},
		Task:     "hailo:model:export",
		Group:    groupRun,
		Short:    "Export a YOLO checkpoint to ONNX",
		Variants: []Variant{{Flag: "all", Task: "hailo:model:export-all", Usage: "every registered model (n/s/m)"}},
	},
	{
		Path:  []string{"hailo", "model", "inspect"},
		Task:  "hailo:model:inspect",
		Group: groupRun,
		Short: "Inspect an ONNX model's graph and I/O",
	},
	{
		Path:  []string{"hailo", "check"},
		Task:  "hailo:check",
		Group: groupCheck,
		Short: "Format check + lint + typecheck + unit tests",
	},
	{Path: []string{"hailo", "test"}, Task: "hailo:test:unit", Group: groupCheck, Short: "Run the pytest unit suite"},
	{
		Path:     []string{"hailo", "lint"},
		Task:     "hailo:lint",
		Group:    groupCheck,
		Short:    "Lint the toolchain (ruff)",
		Variants: []Variant{fixVariant("hailo:lint:fix")},
	},
	{
		Path:  []string{"hailo", "typecheck"},
		Task:  "hailo:typecheck",
		Group: groupCheck,
		Short: "mypy over the toolchain",
	},
	{Path: []string{"hailo", "format"}, Task: "hailo:format", Group: groupCheck, Short: "Format the toolchain (ruff)"},
	{
		Path:  []string{"hailo", "install"},
		Task:  "hailo:env:setup",
		Group: groupSetup,
		Short: "Install the Python dependencies (uv)",
	},
	{
		Path:  []string{"hailo", "packages"},
		Task:  "hailo:env:list",
		Group: groupRun,
		Short: "Show the installed Python packages",
	},
	{
		Path:      []string{"hailo", "clean"},
		Task:      "hailo:clean:all",
		Group:     groupClean,
		Short:     "Remove every generated file",
		Platforms: []Platform{platformLinux, platformDarwin},
		Variants: []Variant{
			{Flag: "output", Task: "hailo:clean:output", Usage: "only the test output and exported models"},
			{Flag: "calib", Task: "hailo:clean:calib", Usage: "only the downloaded calibration data"},
		},
	},
}

// hailoExclusions are the hailo tasks left out of the tree: the Docker suite,
// the ONNX->HEF compile/profile/eval pipeline and the GMR and accuracy sweeps.
var hailoExclusions = map[string]string{
	"hailo:default":                 "Task's own task list; `vt task` replaces it",
	"hailo:docker:dry":              "Hailo Docker suite; use vt task",
	"hailo:docker:load":             "Hailo Docker suite; use vt task",
	"hailo:docker:logs":             "Hailo Docker suite; use vt task",
	"hailo:docker:run":              "Hailo Docker suite; use vt task",
	"hailo:docker:run-compile-only": "Hailo Docker suite; use vt task",
	"hailo:docker:status":           "Hailo Docker suite; use vt task",
	"hailo:docker:stop":             "Hailo Docker suite; use vt task",
	"hailo:compile:dry":             "ONNX to HEF compile; use vt task",
	"hailo:compile:run":             "ONNX to HEF compile; use vt task",
	"hailo:profile:dry":             "HEF profiling; use vt task",
	"hailo:profile:run":             "HEF profiling; use vt task",
	"hailo:eval:dry":                "model evaluation; use vt task",
	"hailo:eval:run":                "model evaluation; use vt task",
	"hailo:eval:visual":             "model evaluation; use vt task",
	"hailo:test:onnx":               "inference smoke test; use vt task",
	"hailo:test:run":                "inference smoke test; use vt task",
	"hailo:test:ultraonnx":          "inference smoke test; use vt task",
	"hailo:model:export-inspect":    "export + inspect pair; use vt task",
	"hailo:calib:convert":           "calibration pipeline; use vt task",
	"hailo:calib:download":          "calibration pipeline; use vt task",
	"hailo:calib:prepare":           "calibration pipeline; use vt task",
	"hailo:stage:no-calib":          "Docker staging; use vt task",
	"hailo:stage:run":               "Docker staging; use vt task",
	"hailo:gmr:compile":             "GMR pipeline; use vt task",
	"hailo:gmr:compile-dry":         "GMR pipeline; use vt task",
	"hailo:gmr:compile-performance": "GMR pipeline; use vt task",
	"hailo:gmr:export":              "GMR pipeline; use vt task",
	"hailo:gmr:stage":               "GMR pipeline; use vt task",
	"hailo:gmr:stage-eval":          "GMR pipeline; use vt task",
	"hailo:gmr:workflow":            "GMR pipeline; use vt task",
	"hailo:accuracy:all":            "accuracy sweep; use vt task",
	"hailo:accuracy:compare":        "accuracy sweep; use vt task",
	"hailo:accuracy:float":          "accuracy sweep; use vt task",
	"hailo:accuracy:sync":           "accuracy sweep; use vt task",
	"hailo:accuracy:sync-validate":  "accuracy sweep; use vt task",
	"hailo:workflow:compile":        "multi-step container workflow; use vt task",
	"hailo:workflow:eval":           "multi-step container workflow; use vt task",
	"hailo:workflow:full":           "multi-step container workflow; use vt task",
	"hailo:log:run":                 "log-level wrapper; use vt task",
}
