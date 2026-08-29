// Command sim-runner orchestrates N scenario runs concurrently (Obstacles
// and Open corpora) against the existing Python simulator, for sim-corpus
// parity testing against the Python baseline (see
// docs/internal/plans/go-migration-plan.md, "Simulation" and "Testing"
// sections). It does not reimplement any simulation math: each scenario run
// is a subprocess call into scripts/sim/run_scenario.py — see
// internal/sim/scenario.SubprocessRunner. Only the orchestration
// (concurrency, aggregation, reporting) is Go-native today, per the plan's
// explicit split between "port the orchestrator now" and "port the core
// math only if profiling says so" (not yet run).
package main

import (
	"context"
	"encoding/json"
	"fmt"
	"io"
	"log/slog"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/sim/scenario"
)

// cliConfig holds every flag sim-runner accepts. Nothing about which
// interpreter/script/corpus/concurrency to use is hardcoded — a deployment
// (dev box vs. CI vs. a future pixi-managed runner) may reasonably want to
// override any of it.
type cliConfig struct {
	corpusPath  string
	command     string
	baseArgs    string
	scriptPath  string
	workDir     string
	pythonPath  string
	extraArgs   string
	concurrency int
	timeout     time.Duration
	jsonOutput  bool
}

// exit codes: 0 means the orchestrator successfully produced a report, even
// one full of scored scenario failures (collisions, timeouts, ...) — those
// are data, not a tool error, matching run_scenario.py's own exit-code
// contract. 1 means the run itself couldn't be completed (bad flags, corpus
// load failure, orchestration-level cancellation). 2 is cobra's own convention
// for a flag/usage error (arg parsing failure, missing required flag) — see
// cli-tool-architect's "misuse vs failure" distinction.
const (
	exitOK    = 0
	exitError = 1
)

// newRootCmd builds the sim-runner cobra command. Flags bind directly into
// cfg; validation of required flags is declarative (cobra's
// MarkFlagRequired) rather than hand-rolled per-field checks.
func newRootCmd(cfg *cliConfig, logger *slog.Logger, stdout io.Writer) *cobra.Command {
	cmd := &cobra.Command{
		Use:   "sim-runner",
		Short: "Run a scenario corpus concurrently against the Python simulator",
		Long: "sim-runner orchestrates N scenario runs concurrently (Obstacles and Open corpora)\n" +
			"against the existing Python simulator, for sim-corpus parity testing against the\n" +
			"Python baseline. It does not reimplement any simulation math — each scenario run\n" +
			"is a subprocess call into scripts/sim/run_scenario.py.",
		Example: "  sim-runner --corpus ./corpus --script ../robot/scripts/sim/run_scenario.py \\\n" +
			"    --workdir ../robot --base-args run,-e,dev,python",
		SilenceUsage:  true,
		SilenceErrors: true,
		RunE: func(cmd *cobra.Command, _ []string) error {
			return run(cmd.Context(), logger, *cfg, stdout)
		},
	}

	flags := cmd.Flags()
	flags.StringVar(&cfg.corpusPath, "corpus", "",
		"path to a scenario corpus: a directory of *_metadata.json files, or a single metadata file")
	flags.StringVar(&cfg.command, "command", "python3", "interpreter executable to invoke run_scenario.py with")
	flags.StringVar(&cfg.baseArgs, "base-args", "",
		"comma-separated argv entries inserted before the script path (e.g. for a wrapper like pixi: run,-e,dev,python)")
	flags.StringVar(&cfg.scriptPath, "script", "", "path to scripts/sim/run_scenario.py")
	flags.StringVar(&cfg.workDir, "workdir", "", "working directory to run the script from, normally platform/robot")
	flags.StringVar(&cfg.pythonPath, "python-path", ".", "PYTHONPATH to set for the subprocess")
	flags.StringVar(&cfg.extraArgs, "extra-args", "",
		"comma-separated flags appended to every run_scenario.py invocation (e.g. --sighted,--no-park)")
	flags.IntVar(&cfg.concurrency, "concurrency", 0, "max scenarios run concurrently; 0 means runtime.NumCPU()")
	flags.DurationVar(&cfg.timeout, "timeout", 0, "per-scenario timeout; 0 means the runner's own default")
	flags.BoolVar(&cfg.jsonOutput, "json", false, "print the report as JSON instead of a text summary")

	for _, name := range []string{"corpus", "script", "workdir"} {
		if err := cmd.MarkFlagRequired(name); err != nil {
			// Only reachable if "name" above is misspelled against a flag
			// that was never registered — a programmer error, not a
			// runtime condition, so panic is appropriate here (this runs
			// once at startup, before any user input is processed).
			panic(fmt.Sprintf("sim-runner: MarkFlagRequired(%q): %v", name, err))
		}
	}

	return cmd
}

func splitCSV(raw string) []string {
	if raw == "" {
		return nil
	}
	return strings.Split(raw, ",")
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig, stdout io.Writer) error {
	scenarios, err := corpus.Load(cfg.corpusPath)
	if err != nil {
		return fmt.Errorf("sim-runner: loading corpus: %w", err)
	}
	logger.Info("loaded corpus", "path", cfg.corpusPath, "scenarios", len(scenarios))

	runner, err := scenario.NewSubprocessRunner(scenario.Config{
		Command:    cfg.command,
		BaseArgs:   splitCSV(cfg.baseArgs),
		ScriptPath: cfg.scriptPath,
		WorkDir:    cfg.workDir,
		PythonPath: cfg.pythonPath,
		ExtraArgs:  splitCSV(cfg.extraArgs),
		Timeout:    cfg.timeout,
	})
	if err != nil {
		return fmt.Errorf("sim-runner: %w", err)
	}

	orchestrator, err := scenario.NewOrchestrator(runner, scenario.OrchestratorConfig{Concurrency: cfg.concurrency})
	if err != nil {
		return fmt.Errorf("sim-runner: %w", err)
	}

	report, runErr := orchestrator.Run(ctx, scenarios)
	printReport(stdout, report, cfg.jsonOutput)
	if runErr != nil {
		return fmt.Errorf("sim-runner: %w", runErr)
	}
	if errored := countErrored(report); errored > 0 {
		return fmt.Errorf("sim-runner: %d of %d scenarios failed to produce a result", errored, len(report.Results))
	}
	return nil
}

func countErrored(report scenario.Report) int {
	errored := 0
	for i := range report.Results {
		if report.Results[i].Err != nil {
			errored++
		}
	}
	return errored
}

func printReport(stdout io.Writer, report scenario.Report, asJSON bool) {
	if asJSON {
		encoder := json.NewEncoder(stdout)
		encoder.SetIndent("", "  ")
		if err := encoder.Encode(report); err != nil {
			fmt.Fprintf(os.Stderr, "sim-runner: encoding report: %v\n", err)
		}
		return
	}

	for i := range report.Results {
		sr := &report.Results[i]
		if sr.Err != nil {
			fmt.Fprintf(stdout, "%-24s ERROR   %v\n", sr.Scenario.ID, sr.Err)
			continue
		}
		fmt.Fprintf(
			stdout,
			"%-24s success=%-5t laps=%d/%d collided=%-5t timed_out=%-5t stuck=%-5t pass_side_violation=%-5t t=%.1fs\n",
			sr.Scenario.ID,
			sr.Result.Success,
			sr.Result.LapsCompleted,
			sr.Result.TargetLaps,
			sr.Result.Collided,
			sr.Result.TimedOut,
			sr.Result.Stuck,
			sr.Result.PassSideViolation,
			sr.Result.SimTimeS,
		)
	}

	fmt.Fprintln(stdout, "---")
	for _, count := range report.Summarize().Counts {
		fmt.Fprintf(stdout, "%-20s %d\n", count.Name, count.Count)
	}
}

func main() {
	os.Exit(mainWithExitCode())
}

// mainWithExitCode does the real work of main and returns the process exit
// code instead of calling os.Exit directly, so `defer stop()` (releasing the
// signal.NotifyContext) actually runs before the process exits.
func mainWithExitCode() int {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	var cfg cliConfig
	cmd := newRootCmd(&cfg, logger, os.Stdout)
	cmd.SetArgs(os.Args[1:])

	if err := cmd.ExecuteContext(ctx); err != nil {
		logger.Error("sim-runner run failed", "error", err)
		return exitError
	}
	return exitOK
}
