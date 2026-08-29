package scenario

import (
	"bytes"
	"context"
	"fmt"
	"os"
	"os/exec"
	"strings"
	"time"

	"github.com/go-playground/validator/v10"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
)

// Config configures a SubprocessRunner. Every field is something a
// deployment might reasonably override — which interpreter to invoke
// (bare "python3" on a dev box vs. "pixi" wrapping a managed env in CI),
// where the script and its working directory live, and how long a single
// run is allowed to take — so none of it is hardcoded as a Go const.
type Config struct {
	// Command is the executable to run, e.g. "python3", or "pixi" when
	// BaseArgs supplies the "run -e dev python" wrapping the project's
	// pixi-managed environment requires (see platform/robot/CLAUDE.md's
	// Tooling section).
	Command string `validate:"required"`

	// BaseArgs are argv entries inserted before ScriptPath, e.g.
	// []string{"run", "-e", "dev", "python"} for the pixi-wrapped
	// invocation. Empty when Command is already the interpreter itself.
	BaseArgs []string

	// ScriptPath is the absolute or WorkDir-relative path to
	// scripts/sim/run_scenario.py.
	ScriptPath string `validate:"required"`

	// WorkDir is the directory the process is run from — must be
	// platform/robot, since run_scenario.py's relative imports
	// (shared.*, src.*, scripts.*) and its own sys.path bootstrap assume
	// that working directory.
	WorkDir string `validate:"required"`

	// PythonPath is the value passed as PYTHONPATH, matching the "with
	// PYTHONPATH=." convention every scripts/sim/*.py script documents.
	// Defaults to "." if left empty.
	PythonPath string

	// ExtraArgs are appended after the metadata path on every invocation,
	// e.g. []string{"--sighted"} or []string{"--no-park"} to run every
	// scenario in the corpus under the same non-default arm.
	ExtraArgs []string

	// Timeout bounds a single scenario run. Defaults to defaultTimeout if
	// zero.
	Timeout time.Duration `validate:"gte=0"`
}

// SubprocessRunner implements Runner by shelling out to
// scripts/sim/run_scenario.py once per scenario — see that script's module
// docstring for the stdout/exit-code contract this depends on. It does not
// reimplement any simulation math; it is purely a process-boundary adapter
// around the existing Python simulator.
type SubprocessRunner struct {
	cfg Config
}

// defaultTimeout bounds one scenario run when Config.Timeout is left at its
// zero value. Sized well above the harness's own worst-case budget (300s,
// OBSTACLES_MAX_STEPS * CONTROL_DT — see
// platform/robot/scripts/common/sim_defaults.py) rather than matching it
// exactly, so an ordinary slow-but-finishing run is never killed by this
// timeout instead of by the simulator's own step budget.
const defaultTimeout = 10 * time.Minute

// pythonPathEnvVar is the environment variable run_scenario.py's own
// sys.path.insert bootstrap depends on being unset or "." — see
// scripts/sim/run_scenario.py's module docstring ("with PYTHONPATH=.").
const pythonPathEnvVar = "PYTHONPATH"

// Compile-time assertion that SubprocessRunner satisfies Runner.
var _ Runner = (*SubprocessRunner)(nil)

// NewSubprocessRunner validates cfg and returns a SubprocessRunner.
func NewSubprocessRunner(cfg Config) (*SubprocessRunner, error) {
	if err := validator.New().Struct(cfg); err != nil {
		return nil, fmt.Errorf("scenario: invalid subprocess runner config: %w", err)
	}
	if cfg.PythonPath == "" {
		cfg.PythonPath = "."
	}
	if cfg.Timeout == 0 {
		cfg.Timeout = defaultTimeout
	}
	return &SubprocessRunner{cfg: cfg}, nil
}

// Run invokes run_scenario.py against sc.MetadataPath and parses its stdout
// as a Result. A non-zero exit code (the script's own contract: a harness
// error, not a scored failure — see its module docstring) is reported as an
// error with the process's stderr attached; ctx cancellation or Config.Timeout
// terminate the subprocess the same way exec.CommandContext always does.
func (r *SubprocessRunner) Run(ctx context.Context, sc corpus.Scenario) (Result, error) {
	runCtx, cancel := context.WithTimeout(ctx, r.cfg.Timeout)
	defer cancel()

	args := make([]string, 0, len(r.cfg.BaseArgs)+1+1+len(r.cfg.ExtraArgs))
	args = append(args, r.cfg.BaseArgs...)
	args = append(args, r.cfg.ScriptPath, sc.MetadataPath)
	args = append(args, r.cfg.ExtraArgs...)

	// #nosec G204 -- Command/BaseArgs/ScriptPath come from operator-supplied
	// Config (a deployment's interpreter/script choice), and sc.MetadataPath
	// comes from corpus.Load's filesystem discovery, not from
	// network/user-request input.
	cmd := exec.CommandContext(runCtx, r.cfg.Command, args...)
	cmd.Dir = r.cfg.WorkDir
	cmd.Env = append(os.Environ(), pythonPathEnvVar+"="+r.cfg.PythonPath)

	var stdout, stderr bytes.Buffer
	cmd.Stdout = &stdout
	cmd.Stderr = &stderr

	if err := cmd.Run(); err != nil {
		return Result{}, fmt.Errorf(
			"scenario %s: running %s: %w (stderr: %s)",
			sc.ID, r.cfg.ScriptPath, err, strings.TrimSpace(stderr.String()),
		)
	}

	result, err := parseResult(bytes.TrimSpace(stdout.Bytes()))
	if err != nil {
		return Result{}, fmt.Errorf("scenario %s: %w", sc.ID, err)
	}
	return result, nil
}
