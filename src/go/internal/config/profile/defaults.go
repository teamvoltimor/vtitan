package profile

// DefaultRoundTimeLimitS is the shipped round time limit, used only as the
// fallback when no config root is known (see internal/sim/scenario). Every
// loaded config takes round_time_limit_s from competition_specs.toml; the TOML
// is the single source of values and every shipped file carries every key its
// schema declares.
const DefaultRoundTimeLimitS = 180.0
