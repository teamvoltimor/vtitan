package vtcli

import (
	"context"
	"errors"
	"fmt"
	"os"
	"os/exec"
	"strconv"
)

// ExitError carries a wrapped task's exit status through cobra to main so the
// CLI propagates it untouched. Nothing in vt inspects or rewrites the child's
// output; the code is the whole contract.
type ExitError struct {
	Code int
}

// Error implements error.
func (e *ExitError) Error() string {
	return "task exited with status " + strconv.Itoa(e.Code)
}

// runTask executes `task <name> extra...` from repoRoot with the terminal
// connected directly and returns an *ExitError when the child fails. The task
// name is validated against the loaded inventory before we get here, and it is
// passed as argv, never through a shell.
func runTask(ctx context.Context, repoRoot, name string, extra []string) error {
	args := append([]string{name}, extra...)

	cmd := exec.CommandContext(ctx, taskBinary, args...)
	cmd.Dir = repoRoot
	cmd.Stdin = os.Stdin
	cmd.Stdout = os.Stdout
	cmd.Stderr = os.Stderr

	if err := cmd.Run(); err != nil {
		if exitErr, ok := errors.AsType[*exec.ExitError](err); ok {
			return &ExitError{Code: exitErr.ExitCode()}
		}

		return fmt.Errorf("run task %s: %w", name, err)
	}

	return nil
}
