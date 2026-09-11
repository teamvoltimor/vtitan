package scenario

import (
	"math"
	"slices"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/waypoints"
)

// passSideScorer enforces WRO rule 9.24.5 from the TRUE layout against the
// TRUE pose, porting scenario_simulator/scoring.py's PassSideScorer.
//
// It exists because the Go runner previously read the verdict from
// SignRouter.WrongSideViolations at the END of a run. That is wrong three
// times over: the router is scoring itself in the BELIEVED frame, the round
// never stopped on a violation (so runs banked laps the rules would have
// denied), and ResetForNewLap clears the router's set at every lap boundary,
// erasing every violation before the last. The router's record is still worth
// keeping as a measure of discovery quality -- it just must not be what ends
// a run.
type passSideScorer struct {
	signs      []signrouter.SignSpec
	direction  trackmodel.Direction
	cornerMinM float64
	cornerMaxM float64
	chassisLen float64
	chassisWid float64

	engaged map[int]struct{}
	scored  map[int]struct{}
	wrong   []int
}

// passSideApproachM is the range within which a sign's radius line is worth
// testing at all, matching _PASS_SIDE_APPROACH_M. Only an optimisation -- the
// radius is a line across the corridor, so a chassis meters away is trivially
// not crossing it. Wide enough that no crossing is missed at any speed the
// robot reaches in one tick.
const passSideApproachM = 1.20

// travelNormal is the unit heading the round is driven along in a section,
// mirroring race_tracker.TRAVEL_DIRS. Go had no equivalent table.
func travelNormal(section trackmodel.Section, direction trackmodel.Direction) (nx, ny float64) {
	cw := direction == trackmodel.Clockwise
	switch section {
	case trackmodel.South:
		if cw {
			return -1, 0
		}
		return 1, 0
	case trackmodel.North:
		if cw {
			return 1, 0
		}
		return -1, 0
	case trackmodel.East:
		if cw {
			return 0, -1
		}
		return 0, 1
	case trackmodel.West:
		if cw {
			return 0, 1
		}
		return 0, -1
	}
	return 0, 0
}

func newPassSideScorer(
	signs []signrouter.SignSpec,
	direction trackmodel.Direction,
	cornerMinM, cornerMaxM, chassisLenM, chassisWidM float64,
) *passSideScorer {
	return &passSideScorer{
		signs:      signs,
		direction:  direction,
		cornerMinM: cornerMinM,
		cornerMaxM: cornerMaxM,
		chassisLen: chassisLenM,
		chassisWid: chassisWidM,
		engaged:    map[int]struct{}{},
		scored:     map[int]struct{}{},
	}
}

// resetForNewLap re-arms every sign so the next lap is judged on its own
// crossings, matching the simulator clearing _pass_side_engaged and
// _pass_side_scored at a lap boundary. `wrong` is deliberately NOT cleared:
// a violation already committed does not stop being one because a lap ended,
// and clearing it is exactly the bug this scorer replaces.
func (p *passSideScorer) resetForNewLap() {
	p.engaged = map[int]struct{}{}
	p.scored = map[int]struct{}{}
}

// radiusGeometry returns (depthAxis, ahead, lateralAxis, permitted) for the
// sign's radius line, matching _radius_geometry. `ahead` is +1/-1 along
// depthAxis pointing the way the round is driven, so "past the line" is a
// single signed comparison.
func (p *passSideScorer) radiusGeometry(
	sign signrouter.SignSpec,
) (depthAxis signrouter.Axis, ahead int, lateralAxis signrouter.Axis, permitted int, ok bool) {
	corridor := waypoints.CorridorForPosition(sign.X, sign.Y, p.cornerMinM, p.cornerMaxM)
	lateralAxis, permitted, ok = signrouter.PassSideLateralAxis(corridor, sign.Color, p.direction)
	if !ok {
		return 0, 0, 0, 0, false
	}
	// The radius runs ACROSS the corridor, so the vehicle travels along the
	// OTHER axis; travelNormal gives which way along it.
	nx, ny := travelNormal(corridor, p.direction)
	if lateralAxis == signrouter.AxisY {
		return signrouter.AxisX, signOf(nx), lateralAxis, permitted, true
	}
	return signrouter.AxisY, signOf(ny), lateralAxis, permitted, true
}

func signOf(v float64) int {
	if v > 0 {
		return 1
	}
	return -1
}

// rectCorners is the oriented chassis rectangle's four corners.
func rectCorners(x, y, yaw, length, width float64) [4][2]float64 {
	c, s := math.Cos(yaw), math.Sin(yaw)
	hl, hw := length/2, width/2
	var out [4][2]float64
	i := 0
	for _, dl := range [2]float64{hl, -hl} {
		for _, dw := range [2]float64{hw, -hw} {
			out[i] = [2]float64{x + dl*c - dw*s, y + dl*s + dw*c}
			i++
		}
	}
	return out
}

// violations returns every wrong-side pass committed so far, sorted.
func (p *passSideScorer) violations() []int {
	if len(p.wrong) == 0 {
		return nil
	}
	out := slices.Clone(p.wrong)
	slices.Sort(out)
	return out
}

// check returns the offending sign indices if the run must stop for a
// wrong-side pass, or nil. The offense is COMPLETING a crossing of the sign's
// radius while on the forbidden side (Appendix A section 5); until the line is
// fully crossed the rules explicitly permit the vehicle to fix its side, so
// nothing is decided. "Completely crosses" is a FOOTPRINT test -- the round
// survives until the last corner is past the line.
func (p *passSideScorer) check(x, y, yaw float64) []int {
	if len(p.signs) == 0 {
		return nil
	}
	corners := rectCorners(x, y, yaw, p.chassisLen, p.chassisWid)
	for index, sign := range p.signs {
		if _, done := p.scored[index]; done {
			continue
		}
		if math.Hypot(sign.X-x, sign.Y-y) > passSideApproachM {
			continue
		}
		depthAxis, ahead, lateralAxis, permitted, ok := p.radiusGeometry(sign)
		if !ok {
			continue
		}
		signDepth := sign.Y
		if depthAxis == signrouter.AxisX {
			signDepth = sign.X
		}
		minPast := math.Inf(1)
		for _, c := range corners {
			d := c[1]
			if depthAxis == signrouter.AxisX {
				d = c[0]
			}
			if v := (d - signDepth) * float64(ahead); v < minPast {
				minPast = v
			}
		}
		if minPast <= 0.0 {
			// Still straddling the line, or not there yet -- the rules let the
			// vehicle fix its side from here, so nothing is decided.
			p.engaged[index] = struct{}{}
			continue
		}
		if _, wasEngaged := p.engaged[index]; !wasEngaged {
			// Beyond the radius without this scorer ever having seen the
			// chassis on the approach side, so no crossing HAPPENED here: the
			// vehicle was PLACED beyond the line (the in-bay start sits inside
			// the approach radius of a sign whose radius is already behind the
			// pocket), or a lap boundary cleared the state while the chassis
			// stood just past one.
			//
			// Deliberately NOT marked scored: doing so would consume the sign,
			// and the genuine crossing later in the same lap would go unjudged.
			continue
		}
		p.scored[index] = struct{}{}
		robotLat, signLat := y, sign.Y
		if lateralAxis == signrouter.AxisX {
			robotLat, signLat = x, sign.X
		}
		if robotLat != signLat {
			side := -1
			if robotLat > signLat {
				side = 1
			}
			if side != permitted {
				p.wrong = append(p.wrong, index)
			}
		}
	}
	if len(p.wrong) == 0 {
		return nil
	}
	out := slices.Clone(p.wrong)
	slices.Sort(out)
	return out
}
