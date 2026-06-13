// Command simgen generates randomized WRO 2026 Gazebo SDF world files.
//
// Usage:
//
//	simgen generate [flags]       — generate randomized scenario worlds
//	simgen generate-track [flags] — regenerate the base track SDF template
package main

import (
	"flag"
	"fmt"
	"log/slog"
	"os"
	"path/filepath"

	"voldemorbot/simgen/internal/generate"
	"voldemorbot/simgen/internal/sdf"
	"voldemorbot/simgen/internal/simconfig"
)

func main() {
	slog.SetDefault(slog.New(slog.NewTextHandler(os.Stderr, nil)))

	if len(os.Args) < 2 {
		usage()
		os.Exit(1)
	}

	switch os.Args[1] {
	case "generate":
		runGenerate(os.Args[2:])
	case "generate-track":
		runGenerateTrack(os.Args[2:])
	default:
		fmt.Fprintf(os.Stderr, "unknown command %q\n\n", os.Args[1])
		usage()
		os.Exit(1)
	}
}

func usage() {
	fmt.Fprintln(os.Stderr, "Usage: simgen <command> [flags]")
	fmt.Fprintln(os.Stderr, "")
	fmt.Fprintln(os.Stderr, "Commands:")
	fmt.Fprintln(os.Stderr, "  generate        Generate randomized scenario SDF files")
	fmt.Fprintln(os.Stderr, "  generate-track  Generate the base track SDF template")
}

func runGenerate(args []string) {
	fs := flag.NewFlagSet("generate", flag.ExitOnError)
	challenge := fs.String("challenge", simconfig.DefaultChallengeType, "Challenge type: open|obstacles")
	numScenarios := fs.Int("num-scenarios", simconfig.DefaultNumScenarios, "Number of scenarios to generate")
	outputDir := fs.String("output-dir", simconfig.DefaultOutputDir, "Output directory for scenario files")
	seed := fs.Int64("seed", simconfig.SeedRandom, "Random seed for reproducible generation (-1 = random)")
	deterministic := fs.Bool("deterministic", false, "Use deterministic defaults instead of full randomization")
	_ = fs.Parse(args)

	challengeType, err := parseChallengeType(*challenge)
	if err != nil {
		fmt.Fprintf(os.Stderr, "error: %v\n", err)
		fs.Usage()
		os.Exit(1)
	}

	var seedPtr *int64
	if *seed >= 0 {
		seedPtr = seed
	}

	var strategy generate.Strategy
	if *deterministic {
		strategy = generate.DeterministicDefaults{}
	}

	scenarioDir := filepath.Join(*outputDir, simconfig.FolderScenarios)
	gen, err := generate.NewScenarioGenerator(scenarioDir, challengeType, seedPtr, strategy)
	if err != nil {
		slog.Error("init generator", "err", err)
		os.Exit(1)
	}

	slog.Info("starting generation",
		"challenge", *challenge,
		"num_scenarios", *numScenarios,
		"output_dir", scenarioDir,
		"seed", *seed,
		"deterministic", *deterministic,
	)

	ok, failed := 0, 0
	for i := range *numScenarios {
		worldPath, _, err := gen.CreateScenario(i)
		if err != nil {
			slog.Error("scenario failed", "index", i, "err", err)
			failed++
			continue
		}
		slog.Info("scenario written", "index", i, "path", worldPath)
		ok++
	}
	slog.Info("generation complete", "ok", ok, "failed", failed)
	if failed > 0 {
		os.Exit(1)
	}
}

func runGenerateTrack(args []string) {
	fs := flag.NewFlagSet("generate-track", flag.ExitOnError)
	output := fs.String("output", "worlds/wro_track_2026.sdf", "Output SDF file path")
	_ = fs.Parse(args)

	if err := os.MkdirAll(filepath.Dir(*output), simconfig.DirPermissions); err != nil {
		slog.Error("create output dir", "err", err)
		os.Exit(1)
	}

	f, err := os.Create(*output)
	if err != nil {
		slog.Error("create file", "path", *output, "err", err)
		os.Exit(1)
	}

	root, _ := sdf.GenerateBaseWorld()
	if _, err := root.WriteTo(f); err != nil {
		f.Close()
		slog.Error("write SDF", "err", err)
		os.Exit(1)
	}
	f.Close()
	slog.Info("base track SDF written", "path", *output)
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
