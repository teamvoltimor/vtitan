package scenario

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/sensormodel"
)

// reverseRunScorer enforces WRO rule 9.21 from the TRUE pose, porting
// scoring.py's _check_reverse_run_violation: a vehicle may travel against
// the round direction only within two sections, the one where the change
// happened and the one behind it; once its footprint is completely outside
// both, the round ends.
//
// Judged on VELOCITY, not heading: reversing is legal while the chassis
// still moves the round's way. Two documented simplifications, both
// stricter than the rules and both Python's: a change on a border takes the
// centre's section, and with several changes the first origin holds until
// travel resumes in the round direction.
type reverseRunScorer struct {
	direction  trackmodel.Direction
	cornerMinM float64
	cornerMaxM float64
	chassisLen float64
	chassisWid float64

	haveOrigin bool
	origin     trackmodel.Section
	originStep int
	violated   bool
}

// reverseRunMinSpeedMPS is _OPPOSITE_SPEED_EPS_MPS: slower than this the
// chassis is at rest, travelling no way at all.
const reverseRunMinSpeedMPS = 0.02

// roundOrder is the counterclockwise section order, _ROUND_ORDER.
var roundOrder = [4]trackmodel.Section{trackmodel.South, trackmodel.East, trackmodel.North, trackmodel.West}

// newReverseRunScorer judges the round passSide judges, with its direction,
// track and chassis; nil, never violated, unless the rule is on.
func (r *NativeRunner) newReverseRunScorer(passSide *passSideScorer) *reverseRunScorer {
	if !r.models.Has(sensormodel.ReverseRun) {
		return nil
	}
	return &reverseRunScorer{
		direction:  passSide.direction,
		cornerMinM: passSide.cornerMinM,
		cornerMaxM: passSide.cornerMaxM,
		chassisLen: passSide.chassisLen,
		chassisWid: passSide.chassisWid,
	}
}

// check folds one tick and reports whether the round has just broken 9.21.
// A nil scorer (the rule off) never does.
func (r *reverseRunScorer) check(st kinematics.AckermannState, step int) bool {
	if r == nil || r.violated {
		return false
	}
	if st.V > -reverseRunMinSpeedMPS && st.V < reverseRunMinSpeedMPS {
		return false
	}
	section := waypoints.CorridorForPosition(st.X, st.Y, r.cornerMinM, r.cornerMaxM)
	nx, ny := travelNormal(section, r.direction)
	if st.V*(math.Cos(st.Yaw)*nx+math.Sin(st.Yaw)*ny) >= 0 {
		r.haveOrigin = false
		return false
	}
	if !r.haveOrigin {
		r.haveOrigin, r.origin, r.originStep = true, section, step
	}
	behind := ringStep(r.origin, r.direction, -1)
	for _, corner := range rectCorners(st.X, st.Y, st.Yaw, r.chassisLen, r.chassisWid) {
		in := waypoints.CorridorForPosition(corner[0], corner[1], r.cornerMinM, r.cornerMaxM)
		if in == r.origin || in == behind {
			return false
		}
	}
	r.violated = true
	return true
}

// ringStep moves steps sections along the round: +1 is the next section in
// the direction of travel, -1 the one behind.
func ringStep(section trackmodel.Section, direction trackmodel.Direction, steps int) trackmodel.Section {
	idx := 0
	for i, s := range roundOrder {
		if s == section {
			idx = i
		}
	}
	if direction == trackmodel.Clockwise {
		steps = -steps
	}
	return roundOrder[((idx+steps)%len(roundOrder)+len(roundOrder))%len(roundOrder)]
}

// fill copies the verdict into res; a violation is never a success.
func (r *reverseRunScorer) fill(res *Result) {
	if r == nil || !r.violated {
		return
	}
	res.ReverseRunViolation = true
	res.ReverseRunOriginStep = new(r.originStep)
	res.ReverseRunOriginSection = r.origin.String()
	res.Success = false
}
