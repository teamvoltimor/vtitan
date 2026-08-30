// sign_lane.go ports
// platform/robot/src/navigation/planning/sign_lane.py: the Obstacles-only
// lane planner that rewrites a corridor's straight-segment waypoints onto
// a pass-side lane, instead of only overriding the pursuit target near a
// sign (see the Python module's docstring for the full "why a lane, not
// just a carrot" rationale).

package signrouter

import (
	"math"
	"sort"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// controlPoint is one (depth, lateral) sample of a corridor's lane
// profile, matching _control_points' list[tuple[float, float]] elements.
type controlPoint struct {
	Depth   float64
	Lateral float64
}

// SignLaneParams is the geometry of the lane transition, matching
// SignLaneParams. See Config for the router's own tuning knobs; this is
// deliberately separate (as in Python) since it parameterizes a single
// apply_sign_lanes call rather than the router's per-tick state.
type SignLaneParams struct {
	// LateralOffsetM is the lateral distance from the sign's own center to
	// the lane (m) -- the same magnitude SignRouter deforms by.
	LateralOffsetM float64
	// RampM is the along-corridor distance to transition on and off the
	// lane (m).
	RampM float64
	// HoldM is the along-corridor half-width of the full-offset hold
	// around a sign (m).
	HoldM float64
	// SplitOverlap splits overlapping plateaux at their midpoint instead
	// of letting one dip through another, matching SIGN_LANE_SPLIT_OVERLAP.
	SplitOverlap bool
	// SkipUnsatisfiable drops a sign whose clamped target is on the
	// forbidden side of it, matching SIGN_LANE_SKIP_UNSATISFIABLE.
	SkipUnsatisfiable bool
	// CornerEntryM is how far past the corridor's straight the lane may
	// extend into the corner arcs either side (m); 0.0 confines it to the
	// straight, matching SIGN_LANE_CORNER_ENTRY_M.
	CornerEntryM float64
}

// halfOf divides evenly by two -- named rather than a bare "/ 2.0" per
// this repo's no-magic-numbers convention.
const halfOf = 2.0

// axisCoords returns (lateral, depth) for wp under axis, matching
// _axis_coords.
func axisCoords(wp trackmodel.Waypoint, axis Axis) (lateral, depth float64) {
	if axis == AxisY {
		return wp.Y, wp.X
	}
	return wp.X, wp.Y
}

// rebuildWaypoint rebuilds a waypoint with its lateral coordinate
// replaced, matching _rebuild.
func rebuildWaypoint(wp trackmodel.Waypoint, axis Axis, lateral float64) trackmodel.Waypoint {
	if axis == AxisY {
		return trackmodel.Waypoint{X: wp.X, Y: lateral}
	}
	return trackmodel.Waypoint{X: lateral, Y: wp.Y}
}

// laneSpan is the depth window the lane may rewrite, straight plus any
// borrowed corner, matching _lane_span.
func laneSpan(cornerEntryM float64, cfg Config) (low, high float64) {
	return cfg.TrackCornerMinM - cornerEntryM, cfg.TrackCornerMaxM + cornerEntryM
}

// inLaneSpan reports whether wp is a point this corridor's lane may move,
// matching _in_lane_span. The depth test widens by cornerEntryM to borrow
// corner-arc runway; the LATERAL test does not widen, which is what keeps
// the widening safe -- see _in_lane_span's Python docstring for the traced
// regression a wider lateral test caused.
func inLaneSpan(
	wp trackmodel.Waypoint,
	corridor trackmodel.Section,
	axis Axis,
	cornerEntryM float64,
	cfg Config,
) bool {
	lateral, depth := axisCoords(wp, axis)
	low, high := laneSpan(cornerEntryM, cfg)
	if depth < low || depth > high {
		return false
	}
	if corridor == trackmodel.South || corridor == trackmodel.West {
		return lateral < cfg.TrackCornerMinM
	}
	return lateral > cfg.TrackCornerMaxM
}

// signPlateaux builds one full-offset plateau per sign, spanning the sign's
// own depth, matching the first half of _control_points. Two signs
// requiring OPPOSITE sides produce an S-bend rather than an averaged,
// invalid centerline (see the Python docstring).
func signPlateaux(
	signs []LaneSpec, corridor trackmodel.Section, axis Axis, params SignLaneParams, cfg Config,
) []controlPoint {
	var plateaux []controlPoint
	for _, entry := range signs {
		_, mult, ok := OutwardLateralAxis(entry.Corridor, entry.Spec.Color)
		if !ok {
			continue
		}
		signLateral, signDepth := axisCoords(
			trackmodel.Waypoint{X: entry.Spec.X, Y: entry.Spec.Y},
			axis,
		)
		target := ClampLateral(signLateral+float64(mult)*params.LateralOffsetM, corridor, cfg)
		if params.SkipUnsatisfiable && (target-signLateral)*float64(mult) <= 0.0 {
			// The clamp put this sign's own target on the forbidden side of
			// it, so every point of its plateau would violate the rule it
			// exists to obey. Emit nothing rather than a line that is wrong
			// by construction.
			continue
		}
		plateaux = append(plateaux, controlPoint{Depth: signDepth, Lateral: target})
	}
	if len(plateaux) == 0 {
		return nil
	}
	sort.Slice(plateaux, func(i, j int) bool { return plateaux[i].Depth < plateaux[j].Depth })
	return plateaux
}

// holdPoints expands each sign plateau into a (low, high) pair spanning
// HoldM either side of its depth, splitting overlaps against neighbors when
// params.SplitOverlap is set, then collapses any resulting fold-back so the
// depth sequence stays monotonic. Matches the second half of
// _control_points.
func holdPoints(plateaux []controlPoint, params SignLaneParams) []controlPoint {
	points := make([]controlPoint, 0, 2*len(plateaux))
	for index, plateau := range plateaux {
		low := plateau.Depth - params.HoldM
		high := plateau.Depth + params.HoldM
		if params.SplitOverlap {
			// A plateau nested inside another puts a HOLE in the enclosing
			// sign's hold window -- split at the midpoint so every sign
			// keeps a flat hold over the stretch where it is actually
			// passed (legal WRO geometry never overlaps; see
			// SignLaneParams.SplitOverlap's Python docstring).
			if index > 0 {
				low = math.Max(low, (plateaux[index-1].Depth+plateau.Depth)/halfOf)
			}
			if index+1 < len(plateaux) {
				high = math.Min(high, (plateau.Depth+plateaux[index+1].Depth)/halfOf)
			}
			if high < low {
				low, high = plateau.Depth, plateau.Depth
			}
		}
		points = append(points,
			controlPoint{Depth: low, Lateral: plateau.Lateral},
			controlPoint{Depth: high, Lateral: plateau.Lateral},
		)
	}
	if len(points) == 0 {
		return nil
	}

	sort.Slice(points, func(i, j int) bool { return points[i].Depth < points[j].Depth })
	// Overlapping plateaux would otherwise leave the profile non-monotonic
	// in depth, which interpolate reads as a fold-back. Collapsing ties to
	// a shared depth turns the fold into an instantaneous transition.
	for i := 1; i < len(points); i++ {
		if points[i].Depth < points[i-1].Depth {
			points[i].Depth = points[i-1].Depth
		}
	}
	return points
}

// controlPoints builds the piecewise-linear (depth, lateral) profile for
// one corridor's lane, matching _control_points. Each sign contributes a
// full-offset plateau spanning HoldM either side of its own depth; the
// profile ramps to baseLateral RampM beyond the outermost plateau at each
// end.
func controlPoints(
	signs []LaneSpec,
	corridor trackmodel.Section,
	axis Axis,
	baseLateral float64,
	params SignLaneParams,
	cfg Config,
) []controlPoint {
	plateaux := signPlateaux(signs, corridor, axis, params, cfg)
	if plateaux == nil {
		return nil
	}
	points := holdPoints(plateaux, params)
	if points == nil {
		return nil
	}

	low, high := laneSpan(params.CornerEntryM, cfg)
	entryDepth := math.Min(math.Max(points[0].Depth-params.RampM, low), points[0].Depth)
	exitDepth := math.Max(
		math.Min(points[len(points)-1].Depth+params.RampM, high),
		points[len(points)-1].Depth,
	)

	profile := make([]controlPoint, 0, len(points)+2)
	profile = append(profile, controlPoint{Depth: entryDepth, Lateral: baseLateral})
	profile = append(profile, points...)
	profile = append(profile, controlPoint{Depth: exitDepth, Lateral: baseLateral})
	return profile
}

// interpolate returns the lateral value of profile at depth, matching
// _interpolate. ok is false outside the profile's span.
func interpolate(profile []controlPoint, depth float64) (lateral float64, ok bool) {
	if depth < profile[0].Depth || depth > profile[len(profile)-1].Depth {
		return 0, false
	}
	for i := 0; i < len(profile)-1; i++ {
		d0, l0 := profile[i].Depth, profile[i].Lateral
		d1, l1 := profile[i+1].Depth, profile[i+1].Lateral
		if d0 <= depth && depth <= d1 {
			if d1 == d0 {
				return l1, true
			}
			return l0 + (l1-l0)*(depth-d0)/(d1-d0), true
		}
	}
	return profile[len(profile)-1].Lateral, true
}

// corridorLaneIndices returns the indices of waypoints in this corridor's
// lane span, and separately those on the STRAIGHT only (not borrowed corner
// runway) -- the latter is what baseLateral must be measured over, since an
// arc's lateral coordinate sweeps away from the centerline as it turns and
// would drag a median computed across arc points off it.
func corridorLaneIndices(
	waypoints []trackmodel.Waypoint,
	corridor trackmodel.Section,
	axis Axis,
	params SignLaneParams,
	cfg Config,
) (indices, straight []int) {
	for i, wp := range waypoints {
		if inLaneSpan(wp, corridor, axis, params.CornerEntryM, cfg) {
			indices = append(indices, i)
		}
	}
	for _, i := range indices {
		if inLaneSpan(waypoints[i], corridor, axis, 0.0, cfg) {
			straight = append(straight, i)
		}
	}
	return indices, straight
}

// medianLateral is the corridor's own centerline as PLANNED: the median
// lateral coordinate over sourceIndices (falling back to every in-span
// index when the straight alone is empty, e.g. an all-corner-runway lane).
func medianLateral(waypoints []trackmodel.Waypoint, axis Axis, indices, straight []int) float64 {
	sourceIndices := straight
	if len(sourceIndices) == 0 {
		sourceIndices = indices
	}
	laterals := make([]float64, len(sourceIndices))
	for j, i := range sourceIndices {
		lat, _ := axisCoords(waypoints[i], axis)
		laterals[j] = lat
	}
	sort.Float64s(laterals)
	return laterals[len(laterals)/2]
}

// applyLaneToCorridor rewrites result's waypoints (in place) within one
// corridor's lane span onto that corridor's pass-side lane profile,
// matching apply_sign_lanes' per-corridor loop body.
func applyLaneToCorridor(
	result []trackmodel.Waypoint, corridor trackmodel.Section, corridorSigns []LaneSpec,
	params SignLaneParams, cfg Config,
) {
	axis, _, ok := OutwardLateralAxis(corridor, corridorSigns[0].Spec.Color)
	if !ok {
		return
	}

	indices, straight := corridorLaneIndices(result, corridor, axis, params, cfg)
	if len(indices) == 0 {
		return
	}
	baseLateral := medianLateral(result, axis, indices, straight)

	profile := controlPoints(corridorSigns, corridor, axis, baseLateral, params, cfg)
	if profile == nil {
		return
	}

	for _, i := range indices {
		lateral, depth := axisCoords(result[i], axis)
		lane, laneOK := interpolate(profile, depth)
		if !laneOK {
			continue
		}
		// Apply the profile as a SHIFT from the centerline, not as an
		// absolute lateral -- on borrowed corner runway an arc point's own
		// lateral is partway through the turn, and an absolute value would
		// snap it back onto the centerline, destroying the turn instead of
		// offsetting it.
		shifted := ClampLateral(lateral+(lane-baseLateral), corridor, cfg)
		if shifted == lateral {
			continue
		}
		result[i] = rebuildWaypoint(result[i], axis, shifted)
	}
}

// ApplySignLanes returns waypointsIn with each signed corridor's straight
// shifted onto its pass-side lane, matching apply_sign_lanes. waypointsIn
// is not mutated. signs is (spec, corridor) for every sign still being
// routed around, corridor as the router itself labels it (SignRouter's
// SignCorridors / LaneSpecs, not recomputed here -- a discovery estimate
// held steady against corner jitter stays steady in the lane too).
//
// Returns a new slice of the same length and order: identical to the
// input when signs is empty, which is every Open Challenge run.
func ApplySignLanes(
	waypointsIn []trackmodel.Waypoint, signs []LaneSpec, params SignLaneParams, cfg Config,
) []trackmodel.Waypoint {
	if len(signs) == 0 || len(waypointsIn) == 0 {
		return append([]trackmodel.Waypoint(nil), waypointsIn...)
	}

	result := append([]trackmodel.Waypoint(nil), waypointsIn...)

	byCorridor := map[trackmodel.Section][]LaneSpec{}
	var corridorOrder []trackmodel.Section
	for _, entry := range signs {
		if _, seen := byCorridor[entry.Corridor]; !seen {
			corridorOrder = append(corridorOrder, entry.Corridor)
		}
		byCorridor[entry.Corridor] = append(byCorridor[entry.Corridor], entry)
	}

	for _, corridor := range corridorOrder {
		applyLaneToCorridor(result, corridor, byCorridor[corridor], params, cfg)
	}

	return result
}
