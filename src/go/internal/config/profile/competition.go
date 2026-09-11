package profile

// CompetitionConfig mirrors platform/config/competition_specs.toml
// (shared.config.constants.CompetitionSpecs): the rule-book numbers a run is
// scored against, as opposed to anything about this robot.
type CompetitionConfig struct {
	// RoundTimeLimitS matches ROUND_TIME_LIMIT_S: a round must finish inside
	// this, so a run that laps after it scores nothing for those laps.
	RoundTimeLimitS float64 `mapstructure:"round_time_limit_s" default:"180.0"`
	// OpenChallengeLaps / ObstacleChallengeLaps match the per-challenge lap
	// targets.
	OpenChallengeLaps     int `mapstructure:"open_challenge_laps"     default:"3"`
	ObstacleChallengeLaps int `mapstructure:"obstacle_challenge_laps" default:"3"`
}

// DefaultCompetitionTOMLPath is where competition_specs.toml lives, relative
// to the repo root.
const DefaultCompetitionTOMLPath = "src/config/competition_specs.toml"

// DefaultRoundTimeLimitS matches CompetitionConfig.RoundTimeLimitS's
// `default` tag (and TOML key) for callers that need the shipped budget
// before a config root is known -- a tagged field cannot be read without
// loading a file. TestCompetitionConfig_DefaultsMatchTags keeps the two in
// step.
const DefaultRoundTimeLimitS = 180.0
