package directionestimator

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"

// Direction re-exports trackmodel.Direction -- see that type's doc comment
// for why it lives there rather than here. A true alias, not a new type,
// so existing callers of directionestimator.Direction/Clockwise/
// Counterclockwise keep working unchanged.
type Direction = trackmodel.Direction

const (
	Clockwise        = trackmodel.Clockwise
	Counterclockwise = trackmodel.Counterclockwise
)
