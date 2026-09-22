// Command vt is the development-machine CLI over the repository Taskfiles. It
// wraps `task` rather than replacing it, so `task X` keeps working throughout
// every phase; see other/docs/development/plan-cli-unificada.md.
package main

import (
	"context"
	"errors"
	"fmt"
	"os"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/vtcli"
)

// exitFailure is used for vt's own errors, distinct from a wrapped task's code.
const exitFailure = 1

func main() {
	os.Exit(run())
}

// run wires the app and maps its result to a process exit code. A wrapped
// task's status is propagated untouched; vt's own failures use exitFailure.
func run() int {
	ui := vtcli.NewUI()
	ctx := context.Background()

	repoRoot, err := vtcli.FindRepoRoot(".")
	if err != nil {
		fmt.Fprintln(os.Stderr, ui.Error(err))

		return exitFailure
	}

	tasks, loadErr := vtcli.LoadTasks(ctx, repoRoot)
	if loadErr != nil {
		fmt.Fprintln(os.Stderr, ui.Error(loadErr))

		return exitFailure
	}

	root := &cobra.Command{
		Use:   "vt",
		Short: "Development CLI over the repository Taskfiles",
	}

	app, appErr := vtcli.NewApp(repoRoot, root, ui, vtcli.CuratedSpec(), tasks)
	if appErr != nil {
		fmt.Fprintln(os.Stderr, ui.Error(appErr))

		return exitFailure
	}

	if execErr := app.Root.ExecuteContext(ctx); execErr != nil {
		var exitErr *vtcli.ExitError
		if errors.As(execErr, &exitErr) {
			return exitErr.Code
		}

		fmt.Fprintln(os.Stderr, ui.Error(execErr))

		return exitFailure
	}

	return 0
}
