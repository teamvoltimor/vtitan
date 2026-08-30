package profile

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// EscapeConfig mirrors the subset of
// platform/shared/config/navigation/escape/escape.toml
// (shared.config.navigation_tuning.escape.EscapeManeuverParams) that
// StuckDetector.from_tuning and CollisionAvoidanceController.from_tuning
// actually read (internal/nav/controllers.StuckDetector/
// CollisionAvoidanceController's Go equivalents). The remaining ~10 fields
// (POSE_TRAIL_MIN_STEP_M, POSE_TRAIL_LEN, SLALOM_*_FRAMES,
// ESCALATE_AFTER_ATTEMPTS, ESCAPE_SIDE_COMMIT_ATTEMPTS, MAX_ESCAPE_FRAMES,
// STUCK_ESCALATION_FRAMES_PER_ATTEMPT) belong to the pose-trail
// retrace-reverse and escalating-escape logic in the not-yet-ported core
// navigator, so they're omitted here rather than mirrored unused.
type EscapeConfig struct {
	// RevSpeed matches REV_SPEED -- reverse speed during escapes (m/s, negative).
	RevSpeed float64 `mapstructure:"rev_speed"`
	// RevSteerDeg matches REV_STEER_DEG -- road-wheel steering angle held
	// while reversing out (deg); converted to a normalised command via
	// RevSteerNorm.
	RevSteerDeg float64 `mapstructure:"rev_steer_deg"`
	// KTurnMinFrames matches K_TURN_MIN_FRAMES -- OBSTACLE-risk K-turn duration.
	KTurnMinFrames int `mapstructure:"k_turn_min_frames"`
	// KTurnMaxFrames matches K_TURN_MAX_FRAMES -- CRITICAL-risk K-turn duration.
	KTurnMaxFrames int `mapstructure:"k_turn_max_frames"`
	// StuckMoveThreshold matches STUCK_MOVE_THRESHOLD -- movement distance
	// threshold (m) below which the robot is considered not moving.
	StuckMoveThreshold float64 `mapstructure:"stuck_move_threshold"`
	// StuckTimeoutFrames matches STUCK_TIMEOUT_FRAMES.
	StuckTimeoutFrames int `mapstructure:"stuck_timeout_frames"`
	// SideCorrectionSteerDeg matches SIDE_CORRECTION_STEER_DEG -- road-wheel
	// angle for a side-threat correction (deg); converted via
	// SideCorrectionSteerNorm.
	SideCorrectionSteerDeg float64 `mapstructure:"side_correction_steer_deg"`
	// SideCorrectionSpeed matches SIDE_CORRECTION_SPEED.
	SideCorrectionSpeed float64 `mapstructure:"side_correction_speed"`
	// SideCorrectionFrames matches SIDE_CORRECTION_FRAMES.
	SideCorrectionFrames int `mapstructure:"side_correction_frames"`
	// StuckConfirmationChecks matches STUCK_CONFIRMATION_CHECKS.
	StuckConfirmationChecks int `mapstructure:"stuck_confirmation_checks"`
	// StuckHistoryFloor matches STUCK_HISTORY_FLOOR.
	StuckHistoryFloor int `mapstructure:"stuck_history_floor"`
	// MinHistoryForDistance matches MIN_HISTORY_FOR_DISTANCE.
	MinHistoryForDistance int `mapstructure:"min_history_for_distance"`
}

// DefaultEscapeTOMLPath is
// platform/shared/config/navigation/escape/escape.toml, relative to the
// repo root. No per-component profile overlays -- pass nil profileNames to
// Load.
const DefaultEscapeTOMLPath = "platform/shared/config/navigation/escape/escape.toml"

// RevSteerNorm converts RevSteerDeg to a normalised actuator command,
// matching EscapeManeuverParams.rev_steer_norm(): the stored value is a
// physical road-wheel angle, so this is where a wider servo produces a
// SMALLER normalised command for the same physical angle, rather than the
// same command meaning a wider angle on different hardware.
func (c EscapeConfig) RevSteerNorm(maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(c.RevSteerDeg*math.Pi/degToRadTurn, maxSteeringAngleRad)
}

// SideCorrectionSteerNorm converts SideCorrectionSteerDeg to a normalised
// actuator command, matching
// EscapeManeuverParams.side_correction_steer_norm().
func (c EscapeConfig) SideCorrectionSteerNorm(maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(
		c.SideCorrectionSteerDeg*math.Pi/degToRadTurn,
		maxSteeringAngleRad,
	)
}
