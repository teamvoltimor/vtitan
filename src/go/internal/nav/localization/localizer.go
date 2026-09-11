package localization

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// LidarLocalizer estimates (x, y) by matching a LIDAR sweep against known
// wall geometry, matching localization.py's LidarLocalizer.
type LidarLocalizer struct {
	walls *trackmodel.TrackWalls
	cfg   Config

	// lastEstimateTimeS, pendingJumpXY and badFitStreak are the state
	// carried between ticks that a re-seed must discard, and ResetTracking
	// exists to discard all three: the streak counts consecutive scans the
	// CURRENT estimate failed to explain, and a re-seed replaces that
	// estimate outright, so the count accrued against the old one says
	// nothing about the new one.
	lastEstimateTimeS *float64
	pendingJumpXY     *trackmodel.Waypoint
	badFitStreak      int

	// relocalizationCount and lastFitCost are diagnostics only, exposed via
	// RelocalizationCount/LastFitCost -- never reset by ResetTracking, which
	// discards tracking continuity, not run-lifetime counters.
	relocalizationCount int
	lastFitCost         *float64

	// freeSpaceGrid is every free-space candidate the global relocalization
	// search considers, matching Python's _free_space_grid: built once, on
	// first use, and cached for the lifetime of this instance -- walls and
	// the grid step are both fixed for as long as this localizer exists.
	freeSpaceGrid []trackmodel.Waypoint

	// Scratch reused across the grid search, which raycasts
	// GridPoints^2 x Passes times per tick (100 at the shipped 5/4). Rebuilding
	// the ray fan and allocating a fresh range slice per candidate dominated
	// the sweep profile; neither outlives the cost() call that consumes it, so
	// both are safe to reuse. Not part of the estimate, so ResetTracking
	// deliberately leaves them alone.
	fan       *trackmodel.RayFan
	predicted []float64
	// fitScratch is fitCost's own reused prediction buffer, kept separate
	// from predicted above so a fitCost call after the grid search (every
	// tick) never aliases a buffer the grid loop might still read.
	fitScratch []float64
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
	l.badFitStreak = 0
}

// RelocalizationCount is how many times the global search has had to
// rescue the estimate, matching LidarLocalizer.relocalization_count.
//
// Diagnostic only. Non-zero means the local search lost the pose and was
// recovered; the value belongs in the debug snapshot because the failure
// it reports (run_20260907_205830) was invisible in every field the
// navigator already published.
func (l *LidarLocalizer) RelocalizationCount() int {
	return l.relocalizationCount
}

// LastFitCost is the mean clipped squared residual (m^2) of the last
// accepted match, matching LidarLocalizer.last_fit_cost. ok is false until
// the first EstimatePosition call that has a scored a fit (i.e. one whose
// bearings matched its ranges).
//
// Diagnostic only. Around 0.010 on a healthy hardware run (measured median
// over two clean 3-lap runs, 2026-09-07), 0.043 on the run whose estimate
// had lost the track.
func (l *LidarLocalizer) LastFitCost() (cost float64, ok bool) {
	if l.lastFitCost == nil {
		return 0, false
	}
	return *l.lastFitCost, true
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

	// A DIFFERENT cost from the search's own cost() above: mean, not sum, and
	// over informative rays only (see fitCost). Comparing this one against an
	// absolute threshold is the whole point of computing it separately --
	// the search's own cost only ever compares candidates against each
	// other on the same sweep, where a constant offset (e.g. from no-return
	// rays) cancels; this one does not get that luxury.
	bestCost := l.fitCost(best.X, best.Y, yaw, rangesM)
	l.lastFitCost = &bestCost

	// The search is a local hill-climb reseeded from priorXY every call, with
	// no independent check on its own output -- SearchRadiusM is sized for
	// search robustness, not as a physical bound. A wrong-but-locally-cheap
	// match can become the next seed and propagate forever. Confirmed on real
	// hardware 2026-08-04: a CCW run's position snapped to an off-track
	// x < 0 during a k-turn escape and stayed there for the rest of the run.
	//
	// Clearance is 0, matching Python's default: this asks only whether the
	// point is physically occupiable, not whether it is safely navigable.
	offTrack := !l.walls.PointInFreeSpace(best.X, best.Y, 0.0)

	// Both symptoms count toward one streak, because both say the same
	// thing: the search is no longer anywhere near the truth. An off-track
	// winner is impossible outright, and a winner whose predicted sweep does
	// not resemble the real one has not explained the scan however cheap it
	// was relative to its neighbors.
	if offTrack || bestCost > l.cfg.RelocalizeCostThreshold {
		l.badFitStreak++
	} else {
		l.badFitStreak = 0
	}

	// A local search reseeded from its own previous answer has no way back
	// once that answer is wrong. This preempts everything below: it can
	// return a rescued position even when off-track was also true this
	// tick, or when the speed guard would otherwise have held the prior.
	if l.badFitStreak >= l.cfg.RelocalizeAfterScans {
		if rescued, ok := l.relocalizeGlobally(yaw, rangesM, bestCost); ok {
			return rescued
		}
	}

	if offTrack {
		l.pendingJumpXY = nil
		return priorXY
	}

	if l.rejectImplausibleSpeed(best, priorXY, dt) {
		return priorXY
	}

	l.pendingJumpXY = nil
	return best
}

// fitCost is the mean clipped squared residual (m^2) at one pose, over real
// returns only, matching LidarLocalizer._fit_cost.
//
// Deliberately not bestCandidate's own cost(): a no-return ray is
// substituted with LidarMaxRangeM and carries no positional information,
// each contributing a full clipped residual whatever the pose. Included,
// those rays add a large offset that swamps the very difference this number
// exists to detect -- measured in Python, counting them compresses the gap
// between a healthy fit and a lost one from 8x to 2x. The search's own cost
// is left alone: it only ever compares candidates against each other on one
// sweep, where a constant offset cancels; this one is compared against an
// absolute threshold, where it does not.
//
// Uses l.fan (built for this tick's anglesRad in EstimatePosition) rather
// than raycasting only the informative rays: raycasting is a pure function
// of pose/yaw/angle, so casting the full fan and then averaging over the
// informative indices produces identical per-ray predictions to casting a
// fan built from just those angles, at the cost of a few wasted rays rather
// than a second fan construction every tick.
func (l *LidarLocalizer) fitCost(x, y, yaw float64, rangesM []float64) float64 {
	offsetX := l.cfg.LidarMountXOffsetM * math.Cos(yaw)
	offsetY := l.cfg.LidarMountXOffsetM * math.Sin(yaw)
	predicted := l.walls.RaycastFan(
		x+offsetX, y+offsetY, yaw, l.fan, l.cfg.LidarMinRangeM, l.cfg.LidarMaxRangeM, l.fitScratch,
	)
	l.fitScratch = predicted

	total, count := 0.0, 0
	for i, p := range predicted {
		if rangesM[i] >= l.cfg.LidarMaxRangeM {
			continue // no-return ray: substituted with max range, no positional information
		}
		residual := math.Min(math.Abs(p-rangesM[i]), l.cfg.ResidualClipM)
		total += residual * residual
		count++
	}
	if count == 0 {
		return 0.0
	}
	return total / float64(count)
}

// relocalizeGlobally re-solves position over the whole track, with no prior
// at all, matching LidarLocalizer._relocalize_globally.
//
// The local search cannot recover from a wrong seed, because it is reseeded
// from its own previous answer every call. This one is not seeded: it
// scores every free-space candidate on the track against the same cost the
// local search uses (fitCost), so the answer does not depend on how wrong
// the estimate had become. Yaw is still taken as given.
//
// The speed guard is deliberately bypassed by the caller: this corrects an
// estimate already known to be wrong, not motion, so the distance it covers
// carries no information about how fast the robot went.
//
// ok is false when the global winner does not fit MATERIALLY better than
// localCost (bestCost > localCost * RelocalizeAcceptRatio), which is the
// case that matters most: a cost above the threshold does not always mean
// the estimate is lost, it can equally mean the WALL MODEL is wrong, and a
// global search against a wrong model finds the best explanation of a track
// that is not there. A wrong model raises the floor for every candidate, so
// the global winner cannot beat the local one by much.
func (l *LidarLocalizer) relocalizeGlobally(
	yaw float64, rangesM []float64, localCost float64,
) (trackmodel.Waypoint, bool) {
	// Either way the streak restarts: the evidence has been acted on, and
	// leaving it at the trigger would re-run this search on every tick.
	l.badFitStreak = 0

	best := trackmodel.Waypoint{}
	bestCost := math.Inf(1)
	for _, candidate := range l.freeSpaceCandidates() {
		if cost := l.fitCost(candidate.X, candidate.Y, yaw, rangesM); cost < bestCost {
			best, bestCost = candidate, cost
		}
	}

	if math.IsInf(bestCost, 1) || bestCost > localCost*l.cfg.RelocalizeAcceptRatio {
		return trackmodel.Waypoint{}, false
	}

	l.pendingJumpXY = nil
	l.relocalizationCount++
	l.lastFitCost = &bestCost
	return best, true
}

// freeSpaceCandidates is every free-space point on a RelocalizeGridStepM
// grid spanning the track's [MinCoord, MaxCoord] square, matching
// LidarLocalizer._free_space_candidates. Built once, on first use, and
// cached: walls and the grid step are both fixed for the lifetime of this
// localizer instance.
//
// The axis is generated by index (minCoord + step*i) rather than repeatedly
// accumulating step in a loop, to avoid float drift widening or narrowing
// the grid over its span -- and the point count is computed with Ceil so the
// axis is inclusive of maxCoord, matching Python's
// np.arange(min, max + step, step).
func (l *LidarLocalizer) freeSpaceCandidates() []trackmodel.Waypoint {
	if l.freeSpaceGrid != nil {
		return l.freeSpaceGrid
	}

	minCoord, maxCoord, step := l.walls.MinCoord(), l.walls.MaxCoord(), l.cfg.RelocalizeGridStepM
	count := int(math.Ceil((maxCoord-minCoord)/step)) + 1

	candidates := make([]trackmodel.Waypoint, 0, count*count)
	for i := range count {
		x := minCoord + step*float64(i)
		for j := range count {
			y := minCoord + step*float64(j)
			if l.walls.PointInFreeSpace(x, y, 0.0) {
				candidates = append(candidates, trackmodel.Waypoint{X: x, Y: y})
			}
		}
	}
	l.freeSpaceGrid = candidates
	return l.freeSpaceGrid
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
