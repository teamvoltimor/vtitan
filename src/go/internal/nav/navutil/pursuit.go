package navutil

import "math"

// wheelbaseHalfDivisor halves the wheelbase in the curvature formula below:
// this chassis steers both axles in opposite directions by the same amount,
// which pivots it about its center instead of the rear axle and doubles the
// yaw rate for a given steering angle, so the effective steering baseline is
// the wheelbase HALVED, not the full wheelbase.
const wheelbaseHalfDivisor = 2.0

// curvatureCoefficient is the "2" in the standard pure-pursuit curvature
// formula curvature = 2*yLocal / lookahead**2 -- a fixed constant of the
// geometry (not a tunable), kept named per this repo's no-magic-numbers
// convention.
const curvatureCoefficient = 2.0

// PurePursuitSteer computes curvature-based pure pursuit steering toward a
// local-frame target, normalised to [-1, 1], matching utils.pure_pursuit_steer.
//
// Standard formulation: curvature = 2*yLocal / lookahead**2, steering angle =
// atan(curvature * wheelbaseM/2), clamped to the chassis's physical steering
// limit. wheelbaseM is halved internally (see wheelbaseHalfDivisor) because
// this chassis steers both axles in counter-phase, confirmed on hardware
// 2026-07-25, doubling the yaw rate a full-wheelbase formula would predict.
//
// Only valid for a target roughly ahead (xLocal > 0) -- the formula gives a
// plausible-looking but wrong result for a target behind the robot; callers
// must handle that case separately (see WaypointController.compute_steering).
//
// Deviation from the Python original: pure_pursuit_steer reads
// RobotSpecs.WHEELBASE as a module-level constant. navutil has no access to
// (and does not import) hardware-profile config, so wheelbaseM is an
// explicit parameter here instead -- callers (WaypointController) thread
// their own configured value through.
func PurePursuitSteer(
	xLocal, yLocal, minLookaheadDist, wheelbaseM, maxSteeringAngle, yawGainCompensation float64,
) float64 {
	if yawGainCompensation <= 0.0 {
		yawGainCompensation = 1.0
	}
	lookahead := max(math.Hypot(xLocal, yLocal), minLookaheadDist)
	curvature := curvatureCoefficient * yLocal / (lookahead * lookahead)
	// Divide by the fraction of predicted yaw the plant actually delivers, so
	// the angle asked for is the one that produces THIS curvature rather than
	// the one a perfect bicycle would need. 1.0 leaves the geometric answer
	// untouched, matching pure_pursuit_steer's own default.
	steerAngle := math.Atan(curvature * wheelbaseM / wheelbaseHalfDivisor / yawGainCompensation)
	steerAngle = Clamp(steerAngle, -maxSteeringAngle, maxSteeringAngle)
	return steerAngle / maxSteeringAngle
}
