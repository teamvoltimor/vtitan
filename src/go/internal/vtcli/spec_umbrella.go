package vtcli

// umbrellaSpec wraps the cross-module verbs at the top of the tree
// (other/tasks/platform.yml). They stay bare, as in Task.
var umbrellaSpec = []Command{
	{Path: []string{"install"}, Task: "install", Short: "Install all dependencies (sim + robot + Go)"},
	{Path: []string{"init", "dev"}, Task: "init:dev", Short: "One-time dev setup (install + lint)"},
	{Path: []string{"test"}, Task: "test", Short: "Run all tests (Python + Go)"},
	{Path: []string{"lint"}, Task: "lint", Short: "Run all linters (ruff, golangci-lint, ESLint, buf)"},
	{Path: []string{"lint", "fix"}, Task: "lint:fix", Short: "Auto-fix lint issues across modules"},
	{Path: []string{"clean"}, Task: "clean", Short: "Remove generated training data"},
	{Path: []string{"clean", "cache"}, Task: "clean:cache", Short: "Remove Python caches"},
	{Path: []string{"clean", "all"}, Task: "clean:all", Short: "Deep clean: training data, caches and build artifacts"},
}
