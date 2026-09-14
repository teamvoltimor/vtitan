package profile

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/navigation/escape"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// RevSteerNorm converts cfg.RevSteerDeg to a normalised actuator command,
// matching EscapeManeuverParams.rev_steer_norm(): the stored value is a
// physical road-wheel angle, so this is where a wider servo produces a
// SMALLER normalised command for the same physical angle, rather than the
// same command meaning a wider angle on different hardware.
//
// It is a free function because the generated DTO it reads carries no
// methods.
func RevSteerNorm(cfg escape.NavigationEscapeEscape, maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(
		cfg.RevSteerDeg*math.Pi/navutil.DegreesPerHalfTurn,
		maxSteeringAngleRad,
	)
}

// SideCorrectionSteerNorm converts cfg.SideCorrectionSteerDeg to a normalised
// actuator command, matching
// EscapeManeuverParams.side_correction_steer_norm().
func SideCorrectionSteerNorm(cfg escape.NavigationEscapeEscape, maxSteeringAngleRad float64) float64 {
	return navutil.SteeringNormFromAngleRad(
		cfg.SideCorrectionSteerDeg*math.Pi/navutil.DegreesPerHalfTurn,
		maxSteeringAngleRad,
	)
}

// Frames converts one of a config's SECOND-valued durations into control
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
