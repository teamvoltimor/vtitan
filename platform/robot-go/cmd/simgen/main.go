// Command simgen generates randomized WRO 2026 Gazebo SDF world files.
//
// Usage:
//
//	simgen generate [flags]                  — generate randomized scenario worlds
//	simgen generate-track [flags]             — regenerate the base track SDF template
//	simgen preview [flags]                    — render SVG top-down preview from metadata JSON
package main

import (
	"fmt"
	"log/slog"
	"os"
	"path/filepath"

	"github.com/shopspring/decimal"
	"github.com/spf13/cobra"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/generate"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/preview"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/sdf"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/simconfig"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/trackconfig"
)

// generatedFile pairs a destination path with the source text to write there.
type generatedFile struct {
	path     string
	contents string
}

func main() {
	slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr, nil)))

	root := &cobra.Command{
		Use:          "simgen",
		Short:        "WRO 2026 Gazebo SDF world generator",
		SilenceUsage: true,
	}
	root.AddCommand(
		generateCmd(),
		generateTrackCmd(),
		generateTrackConstantsCmd(),
		previewCmd(),
	)

	if err := root.Execute(); err != nil {
		os.Exit(1)
	}
}

func generateCmd() *cobra.Command {
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

			var seedPtr *int64
			if seed >= 0 {
				seedPtr = &seed
			}

			var strategy generate.Strategy
			if deterministic {
				strategy = generate.DeterministicDefaults{}
			}

			scenarioDir := filepath.Join(outputDir, simconfig.FolderScenarios)
			gen, err := generate.NewScenarioGenerator(scenarioDir, challengeType, seedPtr, strategy)
			if err != nil {
				return fmt.Errorf("init generator: %w", err)
			}

			slog.Info("starting generation",
				"challenge", challenge,
				"num_scenarios", numScenarios,
				"output_dir", scenarioDir,
				"seed", seed,
				"deterministic", deterministic,
			)

			ok, failed := 0, 0
			for i := range numScenarios {
				worldPath, _, err := gen.CreateScenario(i)
				if err != nil {
					slog.Error("scenario failed", "index", i, "err", err)
					failed++
					continue
				}
				slog.Info("scenario written", "index", i, "path", worldPath)
				ok++

				metaName := fmt.Sprintf("%s%04d%s", simconfig.ScenarioPrefix, i, simconfig.MetadataSuffix)
				metaPath := filepath.Join(scenarioDir, metaName)
				if svgPath, svgErr := preview.GenerateSVG(metaPath, ""); svgErr != nil {
					slog.Warn("preview generation failed", "index", i, "err", svgErr)
				} else {
					slog.Info("preview written", "index", i, "path", svgPath)
				}
			}

			slog.Info("generation complete", "ok", ok, "failed", failed)
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

func generateTrackCmd() *cobra.Command {
	var output string

	cmd := &cobra.Command{
		Use:   "generate-track",
		Short: "Generate the base track SDF template",
		RunE: func(_ *cobra.Command, _ []string) error {
			if err := os.MkdirAll(filepath.Dir(output), simconfig.DirPermissions); err != nil {
				return fmt.Errorf("create output dir: %w", err)
			}
			f, err := os.Create(output)
			if err != nil {
				return fmt.Errorf("create output file: %w", err)
			}

			root, _ := sdf.GenerateBaseWorld()
			if _, err := root.WriteTo(f); err != nil {
				f.Close()
				return fmt.Errorf("write SDF: %w", err)
			}
			f.Close()
			slog.Info("base track SDF written", "path", output)
			return nil
		},
	}

	cmd.Flags().StringVar(&output, "output", "worlds/wro_track_2026.sdf", "Output SDF file path")

	return cmd
}

func generateTrackConstantsCmd() *cobra.Command {
	var (
		config   string
		goOutput string
	)

	cmd := &cobra.Command{
		Use:   "generate-track-constants",
		Short: "Regenerate mat geometry constants from track.toml",
		RunE: func(_ *cobra.Command, _ []string) error {
			// The chassis width is what decides whether a spawn offset fits
			// inside its band, so validation needs it; it comes from the
			// already-generated robot constants, keeping one direction of
			// dependency between the two sources of truth.
			cfg, err := trackconfig.Load(config, decimal.NewFromFloat(simconfig.RobotWidth))
			if err != nil {
				return fmt.Errorf("load track config: %w", err)
			}

			chassisWidth := decimal.NewFromFloat(simconfig.RobotWidth)
			goSrc, err := trackconfig.GenerateGo(cfg, chassisWidth)
			if err != nil {
				return fmt.Errorf("generate go constants: %w", err)
			}

			outputs := []generatedFile{
				{path: goOutput, contents: goSrc},
			}
			for _, out := range outputs {
				if err := os.MkdirAll(filepath.Dir(out.path), simconfig.DirPermissions); err != nil {
					return fmt.Errorf("create output dir for %s: %w", out.path, err)
				}
				if err := os.WriteFile(out.path, []byte(out.contents), simconfig.FilePermissions); err != nil {
					return fmt.Errorf("write %s: %w", out.path, err)
				}
				slog.Info("track constants written", "path", out.path)
			}

			return nil
		},
	}

	cmd.Flags().StringVar(&config, "config", "./config/track.toml", "Path to track.toml source of truth")
	cmd.Flags().StringVar(&goOutput, "go-output",
		"./robot-go/internal/simgen/simconfig/track_constants.gen.go", "Go const block output path")

	return cmd
}

func previewCmd() *cobra.Command {
	var (
		metadata string
		output   string
	)

	cmd := &cobra.Command{
		Use:   "preview",
		Short: "Render SVG top-down preview from a metadata JSON file",
		RunE: func(_ *cobra.Command, _ []string) error {
			if metadata == "" {
				return fmt.Errorf("--metadata is required")
			}
			outPath, err := preview.GenerateSVG(metadata, output)
			if err != nil {
				return fmt.Errorf("generate SVG preview: %w", err)
			}
			slog.Info("SVG preview written", "path", outPath)
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
