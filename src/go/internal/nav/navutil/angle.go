package navutil

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/pkg/geom"
)

// AxisOffsetRad is the signed deviation of yaw from the nearest track axis:
// positive is left of it, negative is right, matching utils.axis_offset_rad.
func AxisOffsetRad(yaw float64) float64 {
	return geom.WrapAngle(yaw - math.Round(yaw/geom.QuarterTurnRad)*geom.QuarterTurnRad)
}

// AxisErrorRad is how far yaw sits from the nearest track axis, always
// non-negative, matching utils.axis_error_rad.
func AxisErrorRad(yaw float64) float64 {
	return math.Abs(AxisOffsetRad(yaw))
}
