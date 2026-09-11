package localization

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// LidarLocalizer estimates (x, y) by matching a LIDAR sweep against known
// wall geometry, matching localization.py's LidarLocalizer.
type LidarLocalizer struct {
	walls *trackmodel.TrackWalls
	cfg   Config

	// lastEstimateTimeS and pendingJumpXY are the only state carried between
	// ticks, and ResetTracking exists to discard both.
	lastEstimateTimeS *float64
	pendingJumpXY     *trackmodel.Waypoint

	// Scratch reused across the grid search, which raycasts
	// GridPoints^2 x Passes times per tick (100 at the shipped 5/4). Rebuilding
	// the ray fan and allocating a fresh range slice per candidate dominated
	// the sweep profile; neither outlives the cost() call that consumes it, so
	// both are safe to reuse. Not part of the estimate, so ResetTracking
	// deliberately leaves them alone.
	fan       *trackmodel.RayFan
	predicted []float64
}

// gridSpan is the width of the search window in units of its radius: the
// grid runs [-radius, +radius], so it spans 2 x radius. Named because it
// appears both when laying out a pass's candidates and when shrinking the
// radius to the spacing that pass just resolved.
const gridSpan = 2.0

// New builds a localizer over walls with the given search parameters.
func New(walls *trackmodel.TrackWalls, cfg Config) *LidarLocalizer {
	return &LidarLocalizer{walls: walls, cfg: cfg}
}

// ResetTracking forgets everything carried between ticks, for a re-seeded
// position.
//
// The speed bound is a statement about motion BETWEEN consecutive estimates.
// When the caller re-seeds position outright -- a new race, or blind
// direction inference overturning the frame every creep-time fix was computed
// in -- there is no such continuity: the held candidate was found in the old
// frame, and the elapsed time since it spans a discontinuity rather than real
// travel. Left in place, the first estimate after a re-seed can have its
// "impossible" jump confirmed by that stale candidate and be accepted
// immediately, which is exactly the corruption the re-seed exists to discard.
func (l *LidarLocalizer) ResetTracking() {
	l.pendingJumpXY = nil
	l.lastEstimateTimeS = nil
}

// EstimatePosition returns the (x, y) that best explains the given sweep, or
// priorXY unchanged when the winner fell outside the track or implied an
// impossible speed that a second tick has not yet confirmed.
//
// yaw is taken as accurate (IMU-fused) and is not searched over. nowS enables
// the speed bound; nil skips it entirely, matching every Python call made
// before that guard existed. The FIRST call also skips it regardless, since
// there is no prior timestamp to measure elapsed time against -- which is
// what lets the deliberate large single-tick correction that absorbs a
// hand-placement error at race start through.
func (l *LidarLocalizer) EstimatePosition(
	priorXY trackmodel.Waypoint,
	yaw float64,
	rangesM []float64,
	anglesRad []float64,
	nowS *float64,
) trackmodel.Waypoint {
	var dt *float64
	if nowS != nil {
		if l.lastEstimateTimeS != nil {
			elapsed := *nowS - *l.lastEstimateTimeS
			dt = &elapsed
		}
		stamp := *nowS
		l.lastEstimateTimeS = &stamp
	}

	// A sweep whose bearings do not match its ranges cannot be scored: Python
	// would raise on the broadcast. Returning the prior is the same "no
	// usable answer this tick" outcome as a rejected match.
	if len(rangesM) == 0 || len(rangesM) != len(anglesRad) {
		return priorXY
	}

	// Built once per tick and reused across every candidate of every pass.
	// Matches() re-checks the bearings rather than assuming they are fixed, so
	// a caller that does change its fan still gets correct ranges.
	if l.fan == nil || !l.fan.Matches(anglesRad) {
		l.fan = trackmodel.NewRayFan(anglesRad)
	}

	best := priorXY
	radius := l.cfg.SearchRadiusM
	n := l.cfg.GridPoints

	for range l.cfg.Passes {
		best = l.bestCandidate(best, radius, n, yaw, rangesM)
		// Refine at the resolution just found, for the next pass.
		if n > 1 {
			radius = gridSpan * radius / float64(n-1)
		}
	}

	// The search is a local hill-climb reseeded from priorXY every call, with
	// no independent check on its own output -- SearchRadiusM is sized for
	// search robustness, not as a physical bound. A wrong-but-locally-cheap
	// match can become the next seed and propagate forever. Confirmed on real
	// hardware 2026-08-04: a CCW run's position snapped to an off-track
	// x < 0 during a k-turn escape and stayed there for the rest of the run.
	//
	// Clearance is 0, matching Python's default: this asks only whether the
	// point is physically occupiable, not whether it is safely navigable.
	if !l.walls.PointInFreeSpace(best.X, best.Y, 0.0) {
		l.pendingJumpXY = nil
		return priorXY
	}

	if l.rejectImplausibleSpeed(best, priorXY, dt) {
		return priorXY
	}

	l.pendingJumpXY = nil
	return best
}

// bestCandidate scores one grid pass and returns its winner.
//
// The grid is walked dx-outer, dy-inner to match the order Python's
// meshgrid(indexing="ij") ravels in, and ties keep the FIRST minimum, matching
// np.argmin -- so an exact tie resolves to the same candidate in both.
func (l *LidarLocalizer) bestCandidate(
	seed trackmodel.Waypoint,
	radius float64,
	n int,
	yaw float64,
	rangesM []float64,
) trackmodel.Waypoint {
	// Predict from where the SENSOR is, not the body center. The C1 sits
	// LidarMountXOffsetM forward of center, flush with the bumper, so a scan
	// taken there cannot be reproduced by casting from the center. Until
	// 2026-08-21 it was, which biased every forward ray by the offset and
	// pulled the fit along the corridor axis. The simulator raycast from the
	// center too, so the two agreed and the error was invisible in sim while
	// present on hardware.
	offsetX := l.cfg.LidarMountXOffsetM * math.Cos(yaw)
	offsetY := l.cfg.LidarMountXOffsetM * math.Sin(yaw)

	best := seed
	bestCost := math.Inf(1)
	for i := range n {
		for j := range n {
			candidate := trackmodel.Waypoint{
				X: seed.X + gridOffset(radius, n, i),
				Y: seed.Y + gridOffset(radius, n, j),
			}
			predicted := l.walls.RaycastFan(
				candidate.X+offsetX, candidate.Y+offsetY, yaw, l.fan,
				l.cfg.LidarMinRangeM, l.cfg.LidarMaxRangeM, l.predicted,
			)
			l.predicted = predicted
			if cost := l.cost(predicted, rangesM); cost < bestCost {
				best, bestCost = candidate, cost
			}
		}
	}
	return best
}

// cost is the clipped sum of squared per-ray residuals.
//
// Clipping rather than summing raw squares is the point: a plain
// least-squares fit is dominated by its worst rays, and the worst rays are
// exactly the ones whose geometry is not in walls -- a traffic sign or
// parking block standing in the beam, or a whole stretch of far wall in the
// wrong place while corridor widths are still being estimated. Those rays
// drag the fit toward a pose that "explains" geometry that does not exist.
// Clipping bounds how far any single ray can pull, so the majority of
// correctly-modeled rays win.
func (l *LidarLocalizer) cost(predicted, measured []float64) float64 {
	total := 0.0
	for i, p := range predicted {
		residual := math.Abs(p - measured[i])
		residual = math.Min(residual, l.cfg.ResidualClipM)
		total += residual * residual
	}
	return total
}

// gridOffset is the i-th of n points spanning [-radius, radius], matching
// np.linspace. With n == 1 linspace yields the low end rather than the
// midpoint, so that case is mirrored explicitly instead of dividing by zero.
func gridOffset(radius float64, n, i int) float64 {
	if n <= 1 {
		return -radius
	}
	return -radius + gridSpan*radius*float64(i)/float64(n-1)
}

// rejectImplausibleSpeed reports whether best implies a physically impossible
// speed and should be held rather than accepted.
//
// A single tick implying impossible speed is held; if the SAME candidate
// (within JumpConfirmToleranceM) wins again on the very next tick it is
// trusted, because a real correction reconverges to nearly the same position
// from an independent scan while an ambiguous flip does not typically repeat
// identically.
func (l *LidarLocalizer) rejectImplausibleSpeed(
	best, priorXY trackmodel.Waypoint,
	dt *float64,
) bool {
	if dt == nil || *dt <= 0 {
		return false
	}
	if best.DistanceTo(priorXY) <= l.cfg.MaxSpeedMPS**dt {
		return false
	}
	if l.pendingJumpXY != nil && best.DistanceTo(*l.pendingJumpXY) <= l.cfg.JumpConfirmToleranceM {
		return false
	}
	held := best
	l.pendingJumpXY = &held
	return true
}
