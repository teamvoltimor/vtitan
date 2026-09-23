package vtcli

// The modules that make up the deploy: the web frontend, the telemetry
// backend, the simulator's scenario generator, and the shared platform code.
// These are first-class parts of the repo with their own toolchains and their
// own build/test/lint cycles, so they get the same typed treatment as go and
// robot rather than being reachable only through the catch-all.
//
// Every domain orders its commands the same way: running it (dev, build),
// then the quality gates (test, lint, typecheck, fmt), then setup and
// maintenance (install, clean). A reader who knows one knows the others.

// frontendSpec wraps the frontend:* tasks (other/apps/frontend/Taskfile.yml).
var frontendSpec = []Command{
	{
		Path:  []string{"frontend", "dev"},
		Task:  "frontend:dev",
		Group: groupRun,
		Short: "Vite dev server on http://localhost:5173",
		Heavy: true,
	},
	{
		Path:  []string{"frontend", "build"},
		Task:  "frontend:build",
		Group: groupRun,
		Short: "Type-check and build for production",
	},
	{
		Path:  []string{"frontend", "preview"},
		Task:  "frontend:preview",
		Group: groupRun,
		Short: "Preview the production build locally",
		Heavy: true,
	},
	{
		Path:  []string{"frontend", "api"},
		Task:  "frontend:api:generate",
		Group: groupRun,
		Short: "Generate the TypeScript API client from the OpenAPI specs",
	},
	{Path: []string{"frontend", "lint"}, Task: "frontend:lint", Group: groupCheck, Short: "Lint the frontend (ESLint)"},
	{
		Path:  []string{"frontend", "typecheck"},
		Task:  "frontend:typecheck",
		Group: groupCheck,
		Short: "Type-check without emitting (tsc -b)",
	},
	{
		Path:  []string{"frontend", "install"},
		Task:  "frontend:install",
		Group: groupSetup,
		Short: "Install frontend dependencies (Node.js)",
	},
	{
		Path:      []string{"frontend", "docker", "build"},
		Task:      "frontend:docker:build",
		Group:     groupRun,
		Short:     "Build the frontend Docker image",
		Platforms: []string{"linux", "windows"},
	},
	{
		Path:      []string{"frontend", "docker", "run"},
		Task:      "frontend:docker:run",
		Group:     groupRun,
		Short:     "Serve the frontend container on http://localhost:8080",
		Platforms: []string{"linux", "windows"},
		Heavy:     true,
	},
}

// backendSpec wraps the backend:* tasks (other/apps/backend/Taskfile.yml), the
// Go telemetry backend. The two code generators sit under `gen` and the two
// module chores under `mod`, so the top level stays the daily verbs.
var backendSpec = []Command{
	{
		Path:  []string{"backend", "dev"},
		Task:  "backend:dev",
		Group: groupRun,
		Short: "Dev mode with synthetic feeds (:8010 HTTP, :9010 gRPC)",
		Heavy: true,
	},
	{
		Path:  []string{"backend", "run"},
		Task:  "backend:run",
		Group: groupRun,
		Short: "Production mode (expects a real robot feed on :9010)",
		Heavy: true,
	},
	{Path: []string{"backend", "build"}, Task: "backend:build", Group: groupRun, Short: "Compile the backend binary"},
	{Path: []string{"backend", "test"}, Task: "backend:test", Group: groupCheck, Short: "Run the backend tests"},
	{
		Path:     []string{"backend", "lint"},
		Task:     "backend:lint",
		Group:    groupCheck,
		Short:    "Lint the backend (golangci-lint)",
		Variants: []Variant{fixVariant("backend:lint:fix")},
	},
	{
		Path:  []string{"backend", "typecheck"},
		Task:  "backend:typecheck",
		Group: groupCheck,
		Short: "Run go vet on the backend",
	},
	{
		Path:  []string{"backend", "fmt"},
		Task:  "backend:fmt",
		Group: groupCheck,
		Short: "Format the backend (goimports + gofmt)",
	},
	{
		Path:     []string{"backend", "align"},
		Task:     "backend:align",
		Group:    groupCheck,
		Short:    "Check Go struct field alignment (betteralign)",
		Variants: []Variant{fixVariant("backend:align:fix")},
	},
	{
		Path:  []string{"backend", "gen", "sqlc"},
		Task:  "backend:sqlc",
		Group: groupRun,
		Short: "Regenerate the DB access layer from the schema",
	},
	{
		Path:     []string{"backend", "gen", "openapi"},
		Task:     "backend:openapi:generate",
		Group:    groupRun,
		Short:    "Regenerate Go types from the context OpenAPI specs",
		Variants: []Variant{{Flag: "validate", Task: "backend:openapi:validate", Usage: "check them instead"}},
	},
	{
		Path:  []string{"backend", "install"},
		Task:  "backend:install",
		Group: groupSetup,
		Short: "Bootstrap the backend: buf deps, proto, sqlc, tidy",
	},
	{
		Path:  []string{"backend", "mod", "download"},
		Task:  "backend:deps",
		Group: groupSetup,
		Short: "Download the backend's Go dependencies",
	},
	{
		Path:  []string{"backend", "mod", "tidy"},
		Task:  "backend:tidy",
		Group: groupSetup,
		Short: "Tidy the backend's Go modules",
	},
	{
		Path:  []string{"backend", "clean"},
		Task:  "backend:clean",
		Group: groupClean,
		Short: "Remove the build cache and the binary",
	},
}

// configSpec wraps the config:* tasks (other/tasks/platform.yml): the one
// source of truth for hardware, navigation and challenge parameters.
var configSpec = []Command{
	{
		Path:  []string{"config", "gen"},
		Task:  "config:gen",
		Group: groupRun,
		Short: "Regenerate the generated config artifacts from the JSON Schemas",
		Variants: []Variant{
			{Flag: "go-dto", Task: "config:gen-go-dto", Usage: "only the Go DTOs"},
			{Flag: "py", Task: "config:gen-py", Usage: "only the Python modules"},
		},
	},
	{
		Path:  []string{"config", "verify"},
		Task:  "config:verify",
		Group: groupCheck,
		Short: "Fail if the generated artifacts are stale",
	},
	{
		Path:  []string{"config", "check"},
		Task:  "config:check",
		Group: groupCheck,
		Short: "Every TOML key is described and every x-journal ref resolves",
	},
	{
		Path:  []string{"config", "validate"},
		Task:  "config:validate",
		Group: groupCheck,
		Short: "Validate the shared TOML tree against its schemas",
	},
	{
		Path:  []string{"config", "lint"},
		Task:  "config:lint",
		Group: groupCheck,
		Short: "Validate config TOML with Taplo",
	},
	{Path: []string{"config", "fmt"}, Task: "config:fmt", Group: groupCheck, Short: "Format config TOML with Taplo"},
}

// workflowSpec wraps the workflow:* tasks: the multi-step pipelines that chain
// the generators and the recorder, kept apart from the single-step domains.
var workflowSpec = []Command{
	{Path: []string{"workflow", "dev"}, Task: "workflow:dev", Group: groupRun, Short: "install → lint → test"},
	{
		Path:  []string{"workflow", "generate"},
		Task:  "workflow:generate",
		Group: groupRun,
		Short: "Generate the track, then both challenges' scenarios",
	},
	{
		Path:  []string{"workflow", "record"},
		Task:  "workflow:record",
		Group: groupRun,
		Short: "Record, convert the bags and extract frames",
	},
}

// cliSpec wraps the cli:* tasks (src/go/Taskfile.yml): how vt itself is
// built, run, tested and linted. Without this domain vt cannot test itself
// through vt, which is the one gap the boundary rule leaves open -- the tasks
// are real build steps (so they live in the Taskfile), but they are also the
// tool you are holding (so the tool should reach them).
var cliSpec = []Command{
	{
		Path:  []string{"cli", "build"},
		Task:  "cli:build",
		Group: groupRun,
		Short: "Compile the vt dev CLI to src/go/bin/vt",
	},
	{
		Path:        []string{"cli", "run"},
		Task:        "cli:run",
		Group:       groupRun,
		Short:       "Run vt from source, without building",
		Passthrough: true,
		// vt's own picker needs the whole terminal, not a pane inside one.
		Terminal: true,
	},
	{
		Path:  []string{"cli", "test"},
		Task:  "cli:test",
		Group: groupCheck,
		Short: "Run vt's tests, including the anti-drift checks",
	},
	{Path: []string{"cli", "lint"}, Task: "cli:lint", Group: groupCheck, Short: "Lint vt (golangci-lint)"},
	{
		Path:  []string{"cli", "completions"},
		Task:  "cli:completions",
		Group: groupRun,
		Short: "Print the tab-completion script for a shell",
		Args:  []Arg{{Name: "shell", Var: "SHELL", Usage: "bash|zsh|fish (default bash)"}},
	},
}
