// Command simgen generates randomized WRO 2026 Gazebo SDF world files.
//
// Usage:
//
//	simgen generate [flags]                  — generate randomized scenario worlds
//	simgen generate-track [flags]             — regenerate the base track SDF template
//	simgen preview [flags]                    — render SVG top-down preview from metadata JSON
package main

import (
	"errors"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"

	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/preview"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/sdf"
	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// defaultConfigRoot is the repository config directory holding track.toml,
// relative to the repo root.
const defaultConfigRoot = "src/config"

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))
	configRoot := defaultConfigRoot

	root := &cobra.Command{
		Use:          "simgen",
		Short:        "WRO 2026 Gazebo SDF world generator",
		SilenceUsage: true,
	}
	root.PersistentFlags().StringVar(
		&configRoot,
		"config-root",
		defaultConfigRoot,
		"repository config root holding track.toml (repo-root relative)",
	)
	root.AddCommand(
		generateCmd(logger, &configRoot),
		generateTrackCmd(logger, &configRoot),
		previewCmd(logger, &configRoot),
	)

	if err := root.Execute(); err != nil {
		os.Exit(1)
	}
}

// loadConfig loads robot.toml (overlaid with the active hardware profiles)
// and track.toml from the config root. The loaded robot supplies the chassis
// width the track's spawn offsets are derived from.
func loadConfig(configRoot string) (*simconfig.Track, *simconfig.Robot, error) {
	robot, err := simconfig.LoadRobot(configRoot, simconfig.ActiveHardwareProfiles())
	if err != nil {
		return nil, nil, fmt.Errorf("load robot: %w", err)
	}
	track, err := simconfig.LoadTrack(configRoot, robot.RobotWidth)
	if err != nil {
		return nil, nil, fmt.Errorf("load track: %w", err)
	}
	return track, robot, nil
}

// writePreview renders and logs the SVG preview for one generated scenario.
func writePreview(
	logger *slog.Logger,
	track *simconfig.Track,
	robot *simconfig.Robot,
	scenarioDir string,
	i int,
) {
	metaName := fmt.Sprintf("%s%04d%s", simconfig.ScenarioPrefix, i, simconfig.MetadataSuffix)
	metaPath := filepath.Join(scenarioDir, metaName)
	if svgPath, svgErr := preview.GenerateSVG(track, robot, metaPath, ""); svgErr != nil {
		logger.Warn("preview generation failed", "index", i, "err", svgErr)
	} else {
		logger.Info("preview written", "index", i, "path", svgPath)
	}
}

func generateCmd(logger *slog.Logger, configRoot *string) *cobra.Command {
	var (
		challenge     string
		numScenarios  int
		outputDir     string
		seed          int64
		deterministic bool
	)

	cmd := &cobra.Command{
		Use:   "generate",
		Short: "Generate randomized scenario SDF files",
		RunE: func(_ *cobra.Command, _ []string) error {
			challengeType, err := parseChallengeType(challenge)
			if err != nil {
				return err
			}

			track, robot, err := loadConfig(*configRoot)
			if err != nil {
				return err
			}

			var seedPtr *int64
			if seed >= 0 {
				seedPtr = &seed
			}

			var strategy generate.Strategy
			if deterministic {
				strategy = generate.DeterministicDefaults{Track: track, Robot: robot}
			}

			scenarioDir := filepath.Join(outputDir, simconfig.FolderScenarios)
			gen, err := generate.NewScenarioGenerator(track, robot, scenarioDir, challengeType, seedPtr, strategy)
			if err != nil {
				return fmt.Errorf("init generator: %w", err)
			}

			logger.Info("starting generation",
				"challenge", challenge,
				"num_scenarios", numScenarios,
				"output_dir", scenarioDir,
				"seed", seed,
				"deterministic", deterministic,
			)

			ok, failed := 0, 0
			for i := range numScenarios {
				worldPath, _, createErr := gen.CreateScenario(i)
				if createErr != nil {
					logger.Error("scenario failed", "index", i, "err", createErr)
					failed++
					continue
				}
				logger.Info("scenario written", "index", i, "path", worldPath)
				ok++

				writePreview(logger, track, robot, scenarioDir, i)
			}

			logger.Info("generation complete", "ok", ok, "failed", failed)
			if failed > 0 {
				return fmt.Errorf("%d/%d scenarios failed", failed, ok+failed)
			}
			return nil
		},
	}

	cmd.Flags().StringVar(&challenge, "challenge", simconfig.DefaultChallengeType, "Challenge type: open|obstacles")
	cmd.Flags().IntVar(&numScenarios, "num-scenarios", simconfig.DefaultNumScenarios, "Number of scenarios to generate")
	cmd.Flags().StringVar(&outputDir, "output-dir", simconfig.DefaultOutputDir, "Output directory for scenario files")
	cmd.Flags().Int64Var(&seed, "seed", simconfig.SeedRandom, "Random seed for reproducible generation (-1 = random)")
	cmd.Flags().BoolVar(&deterministic, "deterministic", false,
		"Use deterministic defaults instead of full randomization")

	return cmd
}

func generateTrackCmd(logger *slog.Logger, configRoot *string) *cobra.Command {
	var output string

	cmd := &cobra.Command{
		Use:   "generate-track",
		Short: "Generate the base track SDF template",
		RunE: func(_ *cobra.Command, _ []string) error {
			track, robot, err := loadConfig(*configRoot)
			if err != nil {
				return err
			}
			if mkdirErr := os.MkdirAll(filepath.Dir(output), simconfig.DirPermissions); mkdirErr != nil {
				return fmt.Errorf("create output dir: %w", mkdirErr)
			}
			f, createErr := os.Create(output)
			if createErr != nil {
				return fmt.Errorf("create output file: %w", createErr)
			}

			root, _ := sdf.GenerateBaseWorld(track, robot)
			if _, writeErr := root.WriteTo(f); writeErr != nil {
				f.Close()
				return fmt.Errorf("write SDF: %w", writeErr)
			}
			f.Close()
			logger.Info("base track SDF written", "path", output)
			return nil
		},
	}

	cmd.Flags().StringVar(&output, "output", "worlds/wro_track_2026.sdf", "Output SDF file path")

	return cmd
}

func previewCmd(logger *slog.Logger, configRoot *string) *cobra.Command {
	var (
		metadata string
		output   string
	)

	cmd := &cobra.Command{
		Use:   "preview",
		Short: "Render SVG top-down preview from a metadata JSON file",
		RunE: func(_ *cobra.Command, _ []string) error {
			if metadata == "" {
				return errors.New("--metadata is required")
			}
			track, robot, err := loadConfig(*configRoot)
			if err != nil {
				return err
			}
			outPath, err := preview.GenerateSVG(track, robot, metadata, output)
			if err != nil {
				return fmt.Errorf("generate SVG preview: %w", err)
			}
			logger.Info("SVG preview written", "path", outPath)
			return nil
		},
	}

	cmd.Flags().StringVar(&metadata, "metadata", "", "Path to *_metadata.json file (required)")
	cmd.Flags().StringVar(&output, "output", "", "Output SVG path (default: derived from --metadata)")

	return cmd
}

func parseChallengeType(s string) (simconfig.ScenarioType, error) {
	switch s {
	case "open":
		return simconfig.ScenarioTypeOpen, nil
	case "obstacles":
		return simconfig.ScenarioTypeObstacles, nil
	default:
		return "", fmt.Errorf("unknown challenge type %q: want open|obstacles", s)
	}
}
