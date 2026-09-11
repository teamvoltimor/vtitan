package profile

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// EscapeConfig mirrors the subset of
// src/config/navigation/escape/escape.toml
// (shared.config.navigation_tuning.escape.EscapeManeuverParams) that
// StuckDetector.from_tuning and CollisionAvoidanceController.from_tuning
// actually read (internal/nav/controllers.StuckDetector/
// CollisionAvoidanceController's Go equivalents). The remaining ~10 fields
// (POSE_TRAIL_MIN_STEP_M, POSE_TRAIL_LEN, SLALOM_*_FRAMES,
// ESCALATE_AFTER_ATTEMPTS, ESCAPE_SIDE_COMMIT_ATTEMPTS, MAX_ESCAPE_S,
// STUCK_ESCALATION_PER_ATTEMPT_S) belong to the pose-trail
// retrace-reverse and escalating-escape logic in the not-yet-ported core
// navigator, so they're omitted here rather than mirrored unused.
type EscapeConfig struct {
	// RevSpeed matches REV_SPEED -- reverse speed during escapes (m/s, negative).
	RevSpeed float64 `mapstructure:"rev_speed"`
	// RevSteerDeg matches REV_STEER_DEG -- road-wheel steering angle held
	// while reversing out (deg); converted to a normalised command via
	// RevSteerNorm.
	RevSteerDeg float64 `mapstructure:"rev_steer_deg"`
	// KTurnMinS matches K_TURN_MIN_S -- OBSTACLE-risk K-turn duration.
	KTurnMinS float64 `mapstructure:"k_turn_min_s"`
	// KTurnMaxS matches K_TURN_MAX_S -- CRITICAL-risk K-turn duration.
	KTurnMaxS float64 `mapstructure:"k_turn_max_s"`
	// StuckMoveThreshold matches STUCK_MOVE_THRESHOLD -- movement distance
	// threshold (m) below which the robot is considered not moving.
	StuckMoveThreshold float64 `mapstructure:"stuck_move_threshold"`
	// StuckTimeoutS matches STUCK_TIMEOUT_S.
	StuckTimeoutS float64 `mapstructure:"stuck_timeout_s"`
	// SideCorrectionSteerDeg matches SIDE_CORRECTION_STEER_DEG -- road-wheel
	// angle for a side-threat correction (deg); converted via
	// SideCorrectionSteerNorm.
	SideCorrectionSteerDeg float64 `mapstructure:"side_correction_steer_deg"`
	// SideCorrectionSpeed matches SIDE_CORRECTION_SPEED.
	SideCorrectionSpeed float64 `mapstructure:"side_correction_speed"`
	// SideCorrectionS matches SIDE_CORRECTION_S.
	SideCorrectionS float64 `mapstructure:"side_correction_s"`
	// StuckConfirmationChecks matches STUCK_CONFIRMATION_CHECKS.
	StuckConfirmationChecks int `mapstructure:"stuck_confirmation_checks"`
	// StuckHistoryFloorS matches STUCK_HISTORY_FLOOR_S.
	StuckHistoryFloorS float64 `mapstructure:"stuck_history_floor_s"`
	// MinHistoryForDistance matches MIN_HISTORY_FOR_DISTANCE. Ships 2; the
	// `default` tag keeps a TOML that omits the key from silently reverting
	// to the zero value (min_history_for_distance = 0 disables the gate).
	MinHistoryForDistance int `mapstructure:"min_history_for_distance" default:"2"`
}

// DefaultEscapeTOMLPath is
// src/config/navigation/escape/escape.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultEscapeTOMLPath = "src/config/navigation/escape/escape.toml"

// RevSteerNorm converts RevSteerDeg to a normalised actuator command,
// matching EscapeManeuverParams.rev_steer_norm(): the stored value is a
// physical road-wheel angle, so this is where a wider servo produces a
// SMALLER normalised command for the same physical angle, rather than the
// same command meaning a wider angle on different hardware.
func (c EscapeConfig) RevSteerNorm(maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(c.RevSteerDeg*math.Pi/navutil.DegreesPerHalfTurn, maxSteeringAngleRad)
}

// SideCorrectionSteerNorm converts SideCorrectionSteerDeg to a normalised
// actuator command, matching
// EscapeManeuverParams.side_correction_steer_norm().
func (c EscapeConfig) SideCorrectionSteerNorm(maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(
		c.SideCorrectionSteerDeg*math.Pi/navutil.DegreesPerHalfTurn,
		maxSteeringAngleRad,
	)
}

// Frames converts one of this config's SECOND-valued durations into control
// ticks at controlHz, mirroring EscapeManeuverParams.frames.
//
// Durations are stored in seconds and converted at the point of use because
// a stored frame count silently means a different duration at a different
// loop rate: 40 frames is 2 s at the shipped 20 Hz and 0.8 s at 50 Hz. Every
// escape length, the stuck timeout and the parking give-up would shift
// together, with nothing raising and no config edited.
//
// Rounds rather than truncates, and floors at one tick: a duration shorter
// than a single tick is still a maneuver the caller asked for, and zero
// frames would skip it entirely.
func Frames(seconds, controlHz float64) int {
	frames := int(math.Round(seconds * controlHz))
	if frames < 1 {
		return 1
	}
	return frames
}
