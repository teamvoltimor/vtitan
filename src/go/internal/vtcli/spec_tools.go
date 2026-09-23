package vtcli

// Small toolchains, contracts and shared assets. Each is a handful of tasks
// rather than a full app, so they share a grouping instinct: they are the
// plumbing every other domain stands on. Wrapped here are the verbs a person
// actually reaches for; the rest stay reachable through `vt task` and are
// excluded by name so the anti-drift check keeps asking for a decision.

// simgenSpec wraps the simgen:* tasks (src/go/Taskfile.yml): the Go scenario
// generator, a real module with its own build and test.
var simgenSpec = []Command{
	{Path: []string{"simgen", "build"}, Task: "simgen:build", Short: "Compile the simgen binary to src/go/bin"},
	{Path: []string{"simgen", "test"}, Task: "simgen:test", Short: "Run the simgen tests"},
	{
		Path:     []string{"simgen", "lint"},
		Task:     "simgen:lint",
		Short:    "Lint simgen (golangci-lint)",
		Variants: []Variant{fixVariant("simgen:lint:fix")},
	},
	{Path: []string{"simgen", "fmt"}, Task: "simgen:fmt", Short: "Format simgen (goimports + golines)"},
	{
		Path:     []string{"simgen", "align"},
		Task:     "simgen:align",
		Short:    "Check Go struct field alignment (betteralign)",
		Variants: []Variant{fixVariant("simgen:align:fix")},
	},
}

// simgenExclusions are the simgen tasks left out: `install` only calls
// `build`, so wrapping both would list the same build twice.
var simgenExclusions = map[string]string{
	"simgen:install": "only calls simgen:build; use vt simgen build",
}

// docsSpec wraps the prose and diagram gates (src/Taskfile.yml and the root
// Taskfile.yml). These run in CI, so they are worth reaching by name.
var docsSpec = []Command{
	{
		Path:  []string{"docs", "check"},
		Task:  "docs:check",
		Short: "Fail if tracked prose points at missing paths or docs",
	},
	{
		Path:  []string{"docs", "adr-refs"},
		Task:  "docs:adr-refs",
		Short: "Fail if a recorded measurement has no adr:NNNN ref",
	},
	{Path: []string{"docs", "mermaid"}, Task: "docs:mermaid", Short: "Refresh the Mermaid blocks inlined in README.md"},
	{
		Path:  []string{"docs", "diagrams"},
		Task:  "docs:diagrams",
		Short: "Render every scheme .mmd to a lossless WebP",
	},
	{
		Path:  []string{"docs", "blueprints"},
		Task:  "docs:blueprints",
		Short: "Re-encode models/*/blueprints to lossy WebP",
	},
}

// sharedSpec wraps the shared Python platform lint (src/python/Taskfile.yml).
var sharedSpec = []Command{
	{
		Path:     []string{"shared", "lint"},
		Task:     "shared:lint",
		Short:    "Lint the shared platform code (ruff)",
		Variants: []Variant{fixVariant("shared:lint:fix")},
	},
}

// contractsSpec wraps the proto and OpenAPI gates (other/tasks/platform.yml):
// the contracts shared with the app frontends.
var contractsSpec = []Command{
	{
		Path:  []string{"proto"},
		Task:  "proto",
		Short: "Buf toolchain ops on the shared proto contract",
		Args:  []Arg{{Name: "action", Var: "ACTION", Required: true, Usage: "update|generate|lint|breaking|format"}},
	},
	{Path: []string{"openapi", "lint"}, Task: "openapi:lint", Short: "Lint the aggregated OpenAPI spec (Redocly)"},
	{Path: []string{"openapi", "bundle"}, Task: "openapi:bundle", Short: "Bundle the aggregated OpenAPI spec"},
	{
		Path:      []string{"openapi", "preview"},
		Task:      "openapi:preview-docs",
		Short:     "Interactive local preview of the API docs",
		Heavy:     true,
		Platforms: []string{"linux", "darwin"},
	},
}

// modelsSpec wraps the tracked-model promotion (other/ml/weights/Taskfile.yml),
// included without flatten, so its names carry the `models:` prefix.
var modelsSpec = []Command{
	{
		Path:  []string{"models", "promote"},
		Task:  "models:promote",
		Short: "Promote a compiled model into a tracked other/ml/weights/<name>/vN/",
		Flags: []Flag{
			{Name: "name", Var: "NAME", Required: true, Usage: "model name"},
			{Name: "hef", Var: "HEF", Required: true, Usage: "compiled .hef file"},
			{Name: "onnx", Var: "ONNX", Usage: "source .onnx file"},
		},
	},
	{
		Path:  []string{"models", "deploy"},
		Task:  "models:deploy",
		Short: "Deploy a tracked model into the auto-annotator ML service",
		Flags: []Flag{
			{Name: "name", Var: "NAME", Required: true, Usage: "model name"},
			{Name: "version", Var: "VERSION", Usage: "vN (default: latest)"},
		},
	},
}

// dockerSpec wraps the repo's docker compose stack (other/tasks/platform.yml).
// The auto-annotator app has its own stack under `annotator`; these are the
// repo-level ones.
var dockerSpec = []Command{
	{
		Path:      []string{"docker", "up"},
		Task:      "docker:up",
		Short:     "Start the Docker Compose services",
		Platforms: []string{"linux", "darwin"},
	},
	{
		Path:      []string{"docker", "down"},
		Task:      "docker:down",
		Short:     "Stop the Docker Compose services",
		Platforms: []string{"linux", "darwin"},
	},
	{
		Path:      []string{"docker", "restart"},
		Task:      "docker:restart",
		Short:     "Restart the Docker Compose services",
		Platforms: []string{"linux", "darwin"},
	},
	{
		Path:      []string{"docker", "logs"},
		Task:      "docker:logs",
		Short:     "Follow the Docker Compose logs",
		Heavy:     true,
		Platforms: []string{"linux", "darwin"},
	},
	{
		Path:      []string{"docker", "build"},
		Task:      "docker:build",
		Short:     "Build the Docker images",
		Platforms: []string{"linux", "darwin"},
	},
}
