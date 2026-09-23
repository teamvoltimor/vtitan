package vtcli

import (
	"fmt"
	"os/exec"
	"strings"
	"testing"
)

// TestSpecTasksExist is anti-drift check 1: every task the spec points at must
// exist in `task --list-all --json`. A rename without a spec update fails here.
func TestSpecTasksExist(t *testing.T) {
	t.Parallel()

	tasks := loadRealTasks(t)

	for _, problem := range checkSpecTasksExist(CuratedSpec(), tasks) {
		t.Error(problem)
	}
}

// TestCuratedDomainsCovered is anti-drift check 2: every task in a curated
// domain is either spec'd or explicitly excluded. It also proves the mechanism
// bites by injecting a task into a curated domain.
func TestCuratedDomainsCovered(t *testing.T) {
	t.Parallel()

	tasks := loadRealTasks(t)

	for _, problem := range checkCuratedCoverage(tasks, CuratedDomains(), CuratedSpec(), Exclusions()) {
		t.Error(problem)
	}

	injected := append(append([]TaskInfo{}, tasks...), TaskInfo{Name: "go:invented"})
	if len(checkCuratedCoverage(injected, CuratedDomains(), CuratedSpec(), Exclusions())) == 0 {
		t.Error("adding an undecided task to a curated domain did not fail the check")
	}
}

// TestCatchAllCoversAll is anti-drift check 3: the catch-all accepts every task
// Task reports and rejects an unknown name.
func TestCatchAllCoversAll(t *testing.T) {
	t.Parallel()

	tasks := loadRealTasks(t)

	for _, task := range tasks {
		if !KnownTask(tasks, task.Name) {
			t.Errorf("catch-all does not accept %q", task.Name)
		}
	}

	if KnownTask(tasks, "this:does:not:exist") {
		t.Error("catch-all accepted an unknown task name")
	}
}

// loadRealTasks returns the live Task inventory, skipping when task is absent
// so `go test` still works in environments without the runner.
func loadRealTasks(t *testing.T) []TaskInfo {
	t.Helper()

	if _, err := exec.LookPath(taskBinary); err != nil {
		t.Skipf("task not installed: %v", err)
	}

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	tasks, err := LoadTasks(t.Context(), root)
	if err != nil {
		t.Fatalf("load tasks: %v", err)
	}

	return tasks
}

// checkSpecTasksExist returns one problem per spec entry with no matching task.
func checkSpecTasksExist(spec []Command, tasks []TaskInfo) []string {
	known := make(map[string]struct{}, len(tasks))
	for _, task := range tasks {
		known[task.Name] = struct{}{}
	}

	problems := make([]string, 0)
	for i := range spec {
		command := &spec[i]
		for _, task := range command.Tasks() {
			if _, ok := known[task]; !ok {
				problems = append(problems, fmt.Sprintf("vt %s points at missing task %q",
					strings.Join(command.Path, " "), task))
			}
		}
	}

	return problems
}

// checkCuratedCoverage returns one problem per curated-domain task that is
// neither covered by the spec nor explicitly excluded.
func checkCuratedCoverage(
	tasks []TaskInfo,
	domains []Domain,
	spec []Command,
	excluded map[string]string,
) []string {
	covered := make(map[string]struct{}, len(spec))
	for i := range spec {
		for _, task := range spec[i].Tasks() {
			covered[task] = struct{}{}
		}
	}

	problems := make([]string, 0)
	for _, task := range tasks {
		domain, owned := ownerDomain(task.Name, domains)
		if !owned {
			continue
		}

		if _, ok := covered[task.Name]; ok {
			continue
		}

		if _, ok := excluded[task.Name]; ok {
			continue
		}

		problems = append(problems,
			fmt.Sprintf("task %q is in curated domain %q but is neither in the spec nor excluded",
				task.Name, domain))
	}

	return problems
}

// TestCommandsGrouped is anti-drift check 10: every command declares one of the
// known groups, so the picker's sections have no unclassified row to fall back
// on.
func TestCommandsGrouped(t *testing.T) {
	t.Parallel()

	known := make(map[CommandGroup]bool, len(commandGroupOrder))
	for _, group := range commandGroupOrder {
		known[group] = true
	}

	spec := CuratedSpec()
	for i := range spec {
		if !known[spec[i].Group] {
			t.Errorf("vt %s has group %q, not one of %v",
				strings.Join(spec[i].Path, " "), spec[i].Group, commandGroupOrder)
		}
	}
}
