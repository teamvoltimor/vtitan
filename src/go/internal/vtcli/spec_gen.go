package vtcli

// challengeFlag picks which challenge a generator targets.
var challengeFlag = Flag{Name: "challenge", Var: "CHALLENGE", Default: "open", Usage: "open|obstacles"}

// metadataArg names a generated scenario by its metadata file.
var metadataArg = Arg{
	Name:  "metadata",
	Var:   "METADATA",
	Usage: "path to a *_metadata.json (default: open scenario 0000)",
}

// genSpec wraps the gen:* tasks: the track, scenarios and the pinned-seed
// sweep corpus (other/tasks/platform.yml), and the gRPC client stubs.
var genSpec = []Command{
	{
		Path:     []string{"gen", "corpus"},
		Task:     "gen:corpus",
		Short:    "The pinned-seed sweep corpus",
		Variants: []Variant{{Flag: "both", Task: "gen:corpus:all", Usage: "both challenges"}},
		Flags: []Flag{
			challengeFlag,
			{Name: "size", Var: "CORPUS_SIZE", Kind: FlagInt, Default: "256", Usage: "scenarios per challenge"},
			{Name: "seed", Var: "CORPUS_SEED", Kind: FlagInt, Default: "2026", Usage: "generator seed"},
		},
	},
	{
		Path:     []string{"gen", "scenarios"},
		Task:     "gen:scenarios",
		Short:    "Random scenarios for a challenge",
		Variants: []Variant{{Flag: "both", Task: "gen:all", Usage: "both challenges"}},
		Flags: []Flag{
			challengeFlag,
			{Name: "count", Var: "SCENARIOS", Kind: FlagInt, Default: "10", Usage: "how many"},
		},
	},
	{Path: []string{"gen", "track"}, Task: "gen:track", Short: "The WRO 2026 base track SDF world"},
	{
		Path:  []string{"gen", "constants"},
		Task:  "gen:track-constants",
		Short: "Go mat-geometry constants from track.toml",
	},
	{
		Path:  []string{"gen", "preview"},
		Task:  "gen:preview",
		Short: "SVG top-down preview of a scenario",
		Args:  []Arg{metadataArg},
	},
	{
		Path:     []string{"gen", "view"},
		Task:     "gen:rviz:all",
		Short:    "A scenario's static layout in RViz (launches RViz too)",
		Heavy:    true,
		Args:     []Arg{metadataArg},
		Flags:    []Flag{profileFlag},
		Variants: []Variant{{Flag: "stream-only", Task: "gen:rviz", Usage: "stream to an RViz already running"}},
	},
	{
		Path:  []string{"gen", "grpc-client"},
		Task:  "gen:robot-command-client",
		Short: "Regenerate src/go's telemetry command gRPC client stubs",
	},
}
