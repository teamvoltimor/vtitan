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
	"errors"
	"fmt"
	"io"
	"log/slog"
	"math"
	"os"
	"os/signal"
	"strings"
	"syscall"
	"time"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/startconditions"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/opencorpus"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/scenario"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensorerrors"
)

// cliConfig holds every flag sim-runner accepts. Nothing about which
// interpreter/script/corpus/concurrency to use is hardcoded — a deployment
// (dev box vs. CI vs. a future pixi-managed runner) may reasonably want to
// override any of it.
type cliConfig struct {
	corpusPath   string
	command      string
	baseArgs     string
	scriptPath   string
	workDir      string
	pythonPath   string
	extraArgs    string
	openSpace    string
	openDir      string
	configRoot   string
	hwProfiles   string
	recordDir    string
	openSeed     uint64
	yawBiasDeg   float64
	imuDriftDPM  float64
	gyroScaleErr float64
	imuNoiseDeg  float64
	startPosErr  float64
	concurrency  int
	timeout      time.Duration
	jsonOutput   bool
	localize     bool
	record       bool
	runner       string
	blind        bool
}

// Runner backend names accepted by --runner.
const (
	runnerPython = "python"
	runnerNative = "native"
)

// secondsPerMinute converts deg/min IMU drift quotes into per-second rates.
const secondsPerMinute = 60.0

// degreesPerHalfCircle is the degrees-per-radians denominator used when
// converting IMU angles quoted in degrees into the radians the model uses.
const degreesPerHalfCircle = 180.0

// Open-space selectors accepted by --open-space. There is no committed Open
// corpus in either language — the space is enumerable from the rules, so it
// is generated rather than stored. See internal/sim/opencorpus.
const (
	openSpaceNone     = ""
	openSpaceFull     = "full"
	openSpace128      = "open128"
	openSpaceBalanced = "balanced128"
)

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
// cfg; which flags are required depends on the others, so the rules live in
// validate rather than in cobra's unconditional MarkFlagRequired.
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
	flags.StringVar(
		&cfg.corpusPath,
		"corpus",
		"",
		"path to a scenario corpus: a directory of *_metadata.json files, or a single metadata file",
	)
	flags.StringVar(
		&cfg.command,
		"command",
		"python3",
		"interpreter executable to invoke run_scenario.py with",
	)
	flags.StringVar(
		&cfg.baseArgs,
		"base-args",
		"",
		"comma-separated argv entries inserted before the script path (e.g. for a wrapper like pixi: run,-e,dev,python)",
	)
	flags.StringVar(&cfg.scriptPath, "script", "", "path to scripts/sim/run_scenario.py")
	flags.StringVar(
		&cfg.workDir,
		"workdir",
		"",
		"working directory to run the script from, normally platform/robot",
	)
	flags.StringVar(&cfg.pythonPath, "python-path", ".", "PYTHONPATH to set for the subprocess")
	flags.StringVar(
		&cfg.extraArgs,
		"extra-args",
		"",
		"comma-separated flags appended to every run_scenario.py invocation (e.g. --sighted,--no-park)",
	)
	flags.IntVar(
		&cfg.concurrency,
		"concurrency",
		0,
		"max scenarios run concurrently; 0 means runtime.NumCPU()",
	)
	flags.DurationVar(
		&cfg.timeout,
		"timeout",
		0,
		"per-scenario timeout; 0 means the runner's own default",
	)
	flags.BoolVar(
		&cfg.jsonOutput,
		"json",
		false,
		"print the report as JSON instead of a text summary",
	)
	flags.StringVar(
		&cfg.runner,
		"runner",
		runnerPython,
		"scenario runner backend: 'python' (subprocess oracle, default) or 'native' (Go-native harness)",
	)
	flags.BoolVar(
		&cfg.localize,
		"localize",
		false,
		"--runner native only: navigate on the LIDAR scan-matcher's pose estimate instead of "+
			"ground truth. This is what ScenarioSimulator does by default (and forces for blind), "+
			"so a like-for-like comparison against the Python oracle needs it",
	)
	flags.BoolVar(
		&cfg.blind,
		"blind",
		false,
		"--runner native only: withhold the scenario's direction and corridor widths, "+
			"so the robot infers both from LIDAR as it does in a real round",
	)

	flags.StringVar(
		&cfg.configRoot,
		"config-root",
		"",
		"--runner native only: repo root to read the shipped TOML tree from; "+
			"empty runs on Go literal defaults, which is NOT the shipped robot "+
			"(base max_mps 0.156, no Open speed ladder)",
	)
	flags.StringVar(
		&cfg.hwProfiles,
		"hardware-profile",
		"",
		"--runner native only: comma-separated hardware profiles to overlay on --config-root, "+
			"one per component (e.g. 270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm)",
	)
	flags.BoolVar(
		&cfg.record,
		"record",
		false,
		"--runner native only: write each scenario's run to an MCAP bag under "+
			"<repo-root>/data/sim/runs/sweep_<stamp>/<scenario>/, openable in Foxglove Studio",
	)
	flags.StringVar(
		&cfg.recordDir,
		"record-dir",
		"",
		"--record only: write the bags here instead of under data/sim/runs",
	)

	// Sensor errors: what the robot is wrong about regarding ITSELF, as
	// opposed to what --blind withholds about the track. All default to
	// zero (a perfect robot), which is the condition every corpus number
	// here was measured on, so switching one on is an explicit A/B.
	flags.Float64Var(
		&cfg.startPosErr,
		"start-pos-error-m",
		0,
		"--runner native only: distance between where the body is and where it believes it is, "+
			"at a random bearing (a robot is set down by hand, not on a surveyed point)",
	)
	flags.Float64Var(
		&cfg.yawBiasDeg,
		"yaw-bias-deg",
		0,
		"--runner native only: constant IMU yaw-zero offset; never corrected, because nothing "+
			"else observes absolute heading",
	)
	flags.Float64Var(
		&cfg.imuDriftDPM,
		"imu-drift-deg-per-min",
		0,
		"--runner native only: IMU yaw drift rate; the BNO085's quoted figure is 0.5",
	)
	flags.Float64Var(
		&cfg.gyroScaleErr,
		"gyro-scale-error",
		0,
		"--runner native only: fractional gyro rotation error (0.005 = 0.5%), accumulated per "+
			"degree TURNED rather than per second",
	)
	flags.Float64Var(
		&cfg.imuNoiseDeg,
		"imu-noise-deg",
		0,
		"--runner native only: per-reading Gaussian yaw noise (stddev)",
	)
	flags.StringVar(
		&cfg.openSpace,
		"open-space",
		openSpaceNone,
		"generate the Open Challenge corpus instead of loading --corpus: "+
			"'full' is the whole 640-case space, 'open128' the legacy start-cell-0 grid",
	)
	flags.Uint64Var(
		&cfg.openSeed,
		"open-space-seed",
		0,
		"--open-space balanced128 only: seed for the start-cell assignment; vary it to confirm "+
			"a result is not an artefact of one spawn assignment",
	)
	flags.StringVar(
		&cfg.openDir,
		"open-space-dir",
		"",
		"--open-space only: directory to materialize the generated scenarios into; "+
			"empty uses a temporary directory removed when the run finishes",
	)

	// --corpus, --script and --workdir are conditionally required, so they
	// are validated in run() rather than with MarkFlagRequired: --corpus is
	// replaced by --open-space, and the two subprocess flags mean nothing to
	// --runner native, which shells out to nothing. Marking them required
	// unconditionally forced a native Open run to invent three paths it
	// would never open.
	return cmd
}

// validate checks the flag combinations cobra cannot: every requirement here
// is conditional on another flag, so MarkFlagRequired (which is
// unconditional) would either under- or over-constrain. Kept separate from
// run so the rules are testable without orchestrating anything.
func validate(cfg cliConfig) error {
	switch cfg.openSpace {
	case openSpaceNone:
		if cfg.corpusPath == "" {
			return errors.New("sim-runner: one of --corpus or --open-space is required")
		}
	case openSpaceFull, openSpace128, openSpaceBalanced:
		if cfg.corpusPath != "" {
			return errors.New("sim-runner: --corpus and --open-space are mutually exclusive")
		}
	default:
		return fmt.Errorf(
			"sim-runner: unknown --open-space %q (want %q, %q or %q)",
			cfg.openSpace, openSpaceFull, openSpaceBalanced, openSpace128,
		)
	}

	switch cfg.runner {
	case runnerNative:
		// Nothing to check: the native runner shells out to nothing, so
		// --script, --workdir, --command, --base-args and --python-path are
		// all inert.
	case runnerPython, "":
		if cfg.scriptPath == "" || cfg.workDir == "" {
			return errors.New("sim-runner: --runner python needs --script and --workdir")
		}
	default:
		return fmt.Errorf("sim-runner: unknown --runner %q (want 'python' or 'native')", cfg.runner)
	}
	return nil
}

// resolveCorpus returns the scenarios to run and a cleanup for anything it
// had to write to disk. Assumes validate has already passed.
func resolveCorpus(logger *slog.Logger, cfg cliConfig) (scenarios []corpus.Scenario, cleanup func(), err error) {
	noop := func() {}

	if cfg.openSpace == openSpaceNone {
		loaded, loadErr := corpus.Load(cfg.corpusPath)
		if loadErr != nil {
			return nil, noop, fmt.Errorf("sim-runner: loading corpus: %w", loadErr)
		}
		logger.Info("loaded corpus", "path", cfg.corpusPath, "scenarios", len(loaded))
		return loaded, noop, nil
	}

	params := opencorpus.Space()
	switch cfg.openSpace {
	case openSpace128:
		params = opencorpus.OuterWallCells(params)
	case openSpaceBalanced:
		balanced, balErr := opencorpus.Balanced128(cfg.openSeed)
		if balErr != nil {
			return nil, noop, fmt.Errorf("sim-runner: %w", balErr)
		}
		params = balanced
	}

	dir := cfg.openDir
	cleanup = noop
	if dir == "" {
		tmp, mkErr := os.MkdirTemp("", "vtitan-open-space-")
		if mkErr != nil {
			return nil, noop, fmt.Errorf("sim-runner: creating a temp corpus directory: %w", mkErr)
		}
		dir = tmp
		cleanup = func() {
			if rmErr := os.RemoveAll(dir); rmErr != nil {
				logger.Warn("removing the generated corpus", "dir", dir, "error", rmErr)
			}
		}
	}

	generated, writeErr := opencorpus.Write(dir, params, startconditions.DefaultConfig())
	if writeErr != nil {
		cleanup()
		return nil, noop, fmt.Errorf("sim-runner: generating the Open corpus: %w", writeErr)
	}
	logger.Info("generated Open corpus", "space", cfg.openSpace, "dir", dir, "scenarios", len(generated))
	return generated, cleanup, nil
}

// sensorErrorsFor converts the CLI's human-facing units into the model's.
// Angles are taken in DEGREES on the command line and drift in deg/min,
// because that is how the BNO085 datasheet quotes them and how anyone
// reasoning about a mount error thinks; the model itself is all radians.
func sensorErrorsFor(cfg cliConfig) sensorerrors.Errors {
	return sensorerrors.Errors{
		StartPosErrorM:  cfg.startPosErr,
		YawBiasRad:      degreesToRadians(cfg.yawBiasDeg),
		IMUDriftRadPerS: degreesToRadians(cfg.imuDriftDPM) / secondsPerMinute,
		GyroScaleError:  cfg.gyroScaleErr,
		IMUNoiseRad:     degreesToRadians(cfg.imuNoiseDeg),
	}
}

func degreesToRadians(deg float64) float64 {
	return deg * math.Pi / degreesPerHalfCircle
}

func splitCSV(raw string) []string {
	if raw == "" {
		return nil
	}
	return strings.Split(raw, ",")
}

func run(ctx context.Context, logger *slog.Logger, cfg cliConfig, stdout io.Writer) error {
	if err := validate(cfg); err != nil {
		return err
	}

	scenarios, cleanup, err := resolveCorpus(logger, cfg)
	if err != nil {
		return err
	}
	defer cleanup()

	var runner scenario.Runner
	switch cfg.runner {
	case runnerNative:
		recordRoot := ""
		if cfg.record {
			root, rootErr := scenario.SimRunsRootFor(cfg.recordDir, time.Now())
			if rootErr != nil {
				return fmt.Errorf("sim-runner: resolving record root: %w", rootErr)
			}
			recordRoot = root
			logger.Info("recording runs", "dir", recordRoot)
		}
		runner = scenario.NewNativeRunner(scenario.NativeRunnerConfig{
			RecordRoot:       recordRoot,
			Blind:            cfg.blind,
			Localize:         cfg.localize,
			ConfigRoot:       cfg.configRoot,
			HardwareProfiles: splitCSV(cfg.hwProfiles),
			SensorErrors:     sensorErrorsFor(cfg),
		})
	case runnerPython, "":
		r, rerr := scenario.NewSubprocessRunner(scenario.Config{
			Command:    cfg.command,
			BaseArgs:   splitCSV(cfg.baseArgs),
			ScriptPath: cfg.scriptPath,
			WorkDir:    cfg.workDir,
			PythonPath: cfg.pythonPath,
			ExtraArgs:  splitCSV(cfg.extraArgs),
			Timeout:    cfg.timeout,
		})
		if rerr != nil {
			return fmt.Errorf("sim-runner: %w", rerr)
		}
		runner = r
	default:
		// Unreachable: validate rejects any other value first.
		return fmt.Errorf("sim-runner: unknown --runner %q (want 'python' or 'native')", cfg.runner)
	}

	orchestrator, err := scenario.NewOrchestrator(
		runner,
		scenario.OrchestratorConfig{Concurrency: cfg.concurrency},
	)
	if err != nil {
		return fmt.Errorf("sim-runner: %w", err)
	}

	report, runErr := orchestrator.Run(ctx, scenarios)
	printReport(stdout, report, cfg.jsonOutput)
	if runErr != nil {
		return fmt.Errorf("sim-runner: %w", runErr)
	}
	if errored := countErrored(report); errored > 0 {
		return fmt.Errorf(
			"sim-runner: %d of %d scenarios failed to produce a result",
			errored,
			len(report.Results),
		)
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
