// routing.go ports
// platform/robot/src/navigation/planning/sign_router/routing.py's
// ROUTING_TABLE and pure helpers (signs_from_metadata excluded -- see
// doc.go).

package signrouter

import (
	"maps"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// RoutingKey is a (corridor, direction) pair, matching ROUTING_TABLE's
// dict key.
type RoutingKey struct {
	Corridor  trackmodel.Section
	Direction trackmodel.Direction
}

// routingTable is the per-(corridor, direction) routing table, matching
// routing.py's ROUTING_TABLE. axis: AxisY means deform the y-coordinate,
// AxisX deforms x. RedMult/GreenMult: +1 or -1 applied to the LATERAL
// offset, chosen so red always moves the deformed waypoint OUTWARD (away
// from the inner square) and green always moves it INWARD -- identically
// for CW and CCW, since outward/inward is a fixed property of the
// corridor, not the travel direction.
var routingTable = map[RoutingKey]RoutingEntry{
	{trackmodel.South, trackmodel.Counterclockwise}: {AxisY, -1, +1},
	{trackmodel.North, trackmodel.Counterclockwise}: {AxisY, +1, -1},
	{trackmodel.East, trackmodel.Counterclockwise}:  {AxisX, +1, -1},
	{trackmodel.West, trackmodel.Counterclockwise}:  {AxisX, -1, +1},
	{trackmodel.South, trackmodel.Clockwise}:        {AxisY, -1, +1},
	{trackmodel.North, trackmodel.Clockwise}:        {AxisY, +1, -1},
	{trackmodel.East, trackmodel.Clockwise}:         {AxisX, +1, -1},
	{trackmodel.West, trackmodel.Clockwise}:         {AxisX, -1, +1},
}

// RoutingTable returns a copy of the per-(corridor, direction) routing
// table, matching ROUTING_TABLE.
func RoutingTable() map[RoutingKey]RoutingEntry {
	out := make(map[RoutingKey]RoutingEntry, len(routingTable))
	maps.Copy(out, routingTable)
	return out
}

// routingEntry looks up routingTable, matching ROUTING_TABLE.get((corridor,
// direction)).
func routingEntry(corridor trackmodel.Section, direction trackmodel.Direction) (RoutingEntry, bool) {
	entry, ok := routingTable[RoutingKey{corridor, direction}]
	return entry, ok
}

// multForColor picks RedMult or GreenMult, the small piece of "which
// multiplier applies" logic every ROUTING_TABLE consumer repeats.
func multForColor(entry RoutingEntry, color SignColor) int {
	if color == SignColorRed {
		return entry.RedMult
	}
	return entry.GreenMult
}

// OutwardLateralAxis returns the world-frame axis and sign of the
// pass-side rule for corridor, direction-agnostic, matching
// outward_lateral_axis. The CLOCKWISE and COUNTERCLOCKWISE rows of
// routingTable are identical for every section (see routingTable's doc
// comment), so looking the rule up under a fixed direction is exactly as
// correct as knowing the real one -- this lets a caller that hasn't
// inferred the travel direction yet still apply "red outward, green
// inward".
//
// Returns (axis, multiplier, true) where a positive multiplier along axis
// points OUTWARD for red, INWARD for green; ok is false if corridor has no
// routing entry.
func OutwardLateralAxis(corridor trackmodel.Section, color SignColor) (axis Axis, multiplier int, ok bool) {
	entry, ok := routingEntry(corridor, trackmodel.Clockwise)
	if !ok {
		return 0, 0, false
	}
	return entry.Axis, multForColor(entry, color), true
}

// ClampLateral clamps a deformed lateral coordinate clear of the inner
// square and outer wall, matching clamp_lateral.
//
// SOUTH/WEST corridors border the inner square on their high side (the
// coordinate must stay below TrackCornerMinM); NORTH/EAST border it on
// their low side (must stay above TrackCornerMaxM). Every corridor is also
// bounded on its outer side by the track wall. This clamp is asymmetric by
// nature -- see clamp_lateral's Python docstring for why rebalancing it has
// been measured and refuted.
func ClampLateral(value float64, corridor trackmodel.Section, cfg Config) float64 {
	wallClearance := cfg.ChassisHalfDiagonalM + cfg.WallClearanceMarginM
	lowSide := corridor == trackmodel.South || corridor == trackmodel.West
	if lowSide {
		value = math.Min(value, cfg.TrackCornerMinM-wallClearance)
		value = math.Max(value, cfg.TrackMinCoordM+wallClearance)
		return value
	}
	value = math.Max(value, cfg.TrackCornerMaxM+wallClearance)
	value = math.Min(value, cfg.TrackMaxCoordM-wallClearance)
	return value
}

// CandidateCorridors returns the corridors a point could plausibly belong
// to, matching candidate_corridors. On a straight this is one section; in
// a CORNER (both coordinates outside the inner square) it is the two
// adjacent faces -- the ambiguity CorridorForPosition resolves by picking
// the nearest one.
func CandidateCorridors(x, y float64, cfg Config) []trackmodel.Section {
	var candidates []trackmodel.Section
	if y < cfg.TrackCornerMinM {
		candidates = append(candidates, trackmodel.South)
	}
	if y > cfg.TrackCornerMaxM {
		candidates = append(candidates, trackmodel.North)
	}
	if x > cfg.TrackCornerMaxM {
		candidates = append(candidates, trackmodel.East)
	}
	if x < cfg.TrackCornerMinM {
		candidates = append(candidates, trackmodel.West)
	}
	return candidates
}

// depthViolation is how far outside its corridor's straight this point's
// DEPTH falls, matching _depth_violation. Zero anywhere along the
// straight; the along-corridor axis is x for SOUTH/NORTH and y for
// EAST/WEST -- the axis OutwardLateralAxis does NOT use, since lateral and
// depth are perpendicular by definition.
func depthViolation(x, y float64, corridor trackmodel.Section, cfg Config) float64 {
	depth := x
	if corridor == trackmodel.East || corridor == trackmodel.West {
		depth = y
	}
	return math.Max(0.0, math.Max(cfg.TrackCornerMinM-depth, depth-cfg.TrackCornerMaxM))
}

// DepthConsistentCorridor returns the candidate face whose straight this
// point actually lies along, matching depth_consistent_corridor.
// CorridorForPosition resolves a corner by NEAREST FACE, which is the
// wrong axis to decide it on for a sign sitting right at a section
// boundary -- see depth_consistent_corridor's Python docstring for the
// measured 42.1% misfile rate this fixes. Ties keep fallback so a genuine
// diagonal is left where CorridorForPosition put it.
func DepthConsistentCorridor(x, y float64, fallback trackmodel.Section, cfg Config) trackmodel.Section {
	candidates := CandidateCorridors(x, y, cfg)
	if len(candidates) < 2 {
		return fallback
	}
	best := candidates[0]
	bestViolation := depthViolation(x, y, best, cfg)
	for _, candidate := range candidates[1:] {
		if v := depthViolation(x, y, candidate, cfg); v < bestViolation {
			best, bestViolation = candidate, v
		}
	}
	if bestViolation < depthViolation(x, y, fallback, cfg) {
		return best
	}
	return fallback
}

// TargetClearance is the clearance this corridor's CLAMPED lane target
// leaves from the sign itself, matching target_clearance. Positive is the
// permitted side; negative means the instruction is unsatisfiable --
// ClampLateral has capped the target at the corridor bound and that bound
// is on the FORBIDDEN side of the sign. ok is false when corridor has no
// routing entry.
func TargetClearance(spec SignSpec, corridor trackmodel.Section, lateralOffsetM float64, cfg Config) (
	clearance float64, ok bool,
) {
	axis, permitted, ok := OutwardLateralAxis(corridor, spec.Color)
	if !ok {
		return 0, false
	}
	lateral := spec.X
	if axis == AxisY {
		lateral = spec.Y
	}
	target := ClampLateral(lateral+float64(permitted)*lateralOffsetM, corridor, cfg)
	return (target - lateral) * float64(permitted), true
}

// SatisfiableCorridor returns corridor, or the corner's OTHER face when
// this one cannot be satisfied, matching satisfiable_corridor. Restricted
// to CandidateCorridors on purpose: any section that makes the arithmetic
// positive would satisfy the check -- including one on the far side of the
// track -- which would plant a geometrically absurd lane while scoring
// well on the metric.
func SatisfiableCorridor(
	spec SignSpec, corridor trackmodel.Section, lateralOffsetM float64, cfg Config,
) trackmodel.Section {
	clearance, ok := TargetClearance(spec, corridor, lateralOffsetM, cfg)
	if !ok || clearance > 0.0 {
		return corridor
	}
	for _, alternative := range CandidateCorridors(spec.X, spec.Y, cfg) {
		if alternative == corridor {
			continue
		}
		if other, otherOK := TargetClearance(spec, alternative, lateralOffsetM, cfg); otherOK && other > 0.0 {
			return alternative
		}
	}
	return corridor
}

// IsSquarelyInCorridor reports whether waypoint (x, y) is still a
// reasonable candidate for straight-corridor deformation, matching
// is_squarely_in_corridor. Two independent checks: the lateral axis (the
// one being overridden) must still read as this corridor, and the depth
// axis (held, never touched) must stay within DeformDepthBufferM of the
// inner square's own span -- wider than CorridorForPosition's own
// classification window on purpose, since the lookahead target runs
// 0.2-0.4m ahead of the robot. See is_squarely_in_corridor's Python
// docstring for why neither an exact window nor no window at all works.
func IsSquarelyInCorridor(x, y float64, corridor trackmodel.Section, cfg Config) bool {
	depthMin := cfg.TrackCornerMinM - cfg.DeformDepthBufferM
	depthMax := cfg.TrackCornerMaxM + cfg.DeformDepthBufferM
	switch corridor {
	case trackmodel.South:
		return y < cfg.TrackCornerMinM && depthMin <= x && x <= depthMax
	case trackmodel.North:
		return y > cfg.TrackCornerMaxM && depthMin <= x && x <= depthMax
	case trackmodel.East:
		return x > cfg.TrackCornerMaxM && depthMin <= y && y <= depthMax
	case trackmodel.West:
		return x < cfg.TrackCornerMinM && depthMin <= y && y <= depthMax
	default:
		return false
	}
}
