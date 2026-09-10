package profile

// CompetitionConfig mirrors platform/config/competition_specs.toml
// (shared.config.constants.CompetitionSpecs): the rule-book numbers a run is
// scored against, as opposed to anything about this robot.
type CompetitionConfig struct {
	// RoundTimeLimitS matches ROUND_TIME_LIMIT_S: a round must finish inside
	// this, so a run that laps after it scores nothing for those laps.
	RoundTimeLimitS float64 `mapstructure:"round_time_limit_s"`
	// OpenChallengeLaps / ObstacleChallengeLaps match the per-challenge lap
	// targets.
	OpenChallengeLaps     int `mapstructure:"open_challenge_laps"`
	ObstacleChallengeLaps int `mapstructure:"obstacle_challenge_laps"`
}

// DefaultCompetitionTOMLPath is where competition_specs.toml lives, relative
// to the repo root.
const DefaultCompetitionTOMLPath = "src/config/competition_specs.toml"

// CompetitionDefaults mirrors the shipped file, for viper to overlay a partial
// TOML onto.
func CompetitionDefaults() map[string]any {
	return map[string]any{
		"round_time_limit_s":      180.0,
		"open_challenge_laps":     3,
		"obstacle_challenge_laps": 3,
	}
}
