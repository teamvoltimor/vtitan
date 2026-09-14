package profile

// DefaultCompetitionTOMLPath is where competition_specs.toml lives, relative
// to the repo root. The file's shape is the generated
// generated.CompetitionSpecs DTO.
const DefaultCompetitionTOMLPath = "src/config/competition_specs.toml"

// DefaultRoundTimeLimitS matches generated.CompetitionSpecs.RoundTimeLimitS's
// registry fallback (and TOML key) for callers that need the shipped budget
// before a config root is known -- a tagged field cannot be read without
// loading a file. TestCompetitionSpecs_DefaultsMatchTags keeps the two in
// step.
const DefaultRoundTimeLimitS = 180.0
