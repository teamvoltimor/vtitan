// router.go ports
// platform/robot/src/navigation/planning/sign_router/router.py's
// SignRouter: the stateful per-tick sign-avoidance router. Discovery mode
// (SignRouter(discover=True), ObservedSignMap ingestion) is not ported --
// see doc.go.

package signrouter

import (
	"math"
	"sort"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/waypoints"
)

// signCandidate is one (index, distance) entry from activeSignCandidates,
// matching the tuples _active_sign_candidates returns.
type signCandidate struct {
	Index int
	Dist  float64
}

// SignRouter routes the robot past traffic signs using lateral waypoint
// deformations, matching router.py's SignRouter class (sighted mode only;
// see doc.go).
type SignRouter struct {
	signs     []SignSpec
	config    Config
	direction trackmodel.Direction

	passed    map[int]struct{}
	engaged   map[int]struct{}
	lapTick   int
	wrongSide map[int]struct{}

	// committed is the sign index currently being routed around, kept
	// across ticks so the commanded line does not jump between two legal
	// ones mid-pass; nil matches Python's committed=None. See
	// preferCommitted.
	committed *int
	// commitYaw is the robot yaw at the tick each sign was first
	// committed to, keyed by sign index.
	commitYaw map[int]float64

	// signCorridors is each sign's own corridor, kept in step with signs
	// -- DeformWaypoint must never apply a sign's (x, y) through a
	// different corridor's axis convention.
	signCorridors []trackmodel.Section
}

// NewSignRouter builds a SignRouter for the given signs, config and travel
// direction, matching SignRouter.__init__ (sighted mode only: no discover/
// discovery_config/tuning parameters -- see doc.go). Returns an error if
// config fails NewConfig's validation.
func NewSignRouter(signs []SignSpec, config Config, direction trackmodel.Direction) (*SignRouter, error) {
	cfg, err := NewConfig(config)
	if err != nil {
		return nil, err
	}
	r := &SignRouter{
		signs:     append([]SignSpec(nil), signs...),
		config:    cfg,
		direction: direction,
		passed:    map[int]struct{}{},
		engaged:   map[int]struct{}{},
		wrongSide: map[int]struct{}{},
		commitYaw: map[int]float64{},
	}
	r.signCorridors = make([]trackmodel.Section, len(r.signs))
	for i, spec := range r.signs {
		r.signCorridors[i] = r.corridorForSpec(spec)
	}
	return r, nil
}

// Signs returns the signs currently being routed around, matching the
// signs property.
func (r *SignRouter) Signs() []SignSpec {
	return append([]SignSpec(nil), r.signs...)
}

// LaneSpecs returns every routed sign paired with the corridor label the
// router uses for it, matching the lane_specs property. Passed signs are
// INCLUDED (the lane is planned geometry for the whole layout, not this
// lap's bookkeeping).
func (r *SignRouter) LaneSpecs() []LaneSpec {
	out := make([]LaneSpec, len(r.signs))
	for i, spec := range r.signs {
		out[i] = LaneSpec{Spec: spec, Corridor: r.signCorridors[i]}
	}
	return out
}

// ActiveSignCount is the number of signs not yet marked as passed this
// lap, matching the active_sign_count property.
func (r *SignRouter) ActiveSignCount() int {
	return len(r.signs) - len(r.passed)
}

// RoutedSignPositions returns the world positions of the signs this
// router still intends to route around, matching the routed_sign_positions
// property. Signs already marked passed are excluded.
func (r *SignRouter) RoutedSignPositions() []trackmodel.Waypoint {
	out := make([]trackmodel.Waypoint, 0, len(r.signs))
	for i, spec := range r.signs {
		if _, isPassed := r.passed[i]; isPassed {
			continue
		}
		out = append(out, trackmodel.Waypoint{X: spec.X, Y: spec.Y})
	}
	return out
}

// RoutedSignPositionsByCorridor is RoutedSignPositions, each paired with
// the sign's own corridor, matching the
// routed_sign_positions_by_corridor property.
func (r *SignRouter) RoutedSignPositionsByCorridor() []RoutedSign {
	out := make([]RoutedSign, 0, len(r.signs))
	for i, ls := range r.LaneSpecs() {
		if _, isPassed := r.passed[i]; isPassed {
			continue
		}
		out = append(out, RoutedSign{Waypoint: trackmodel.Waypoint{X: ls.Spec.X, Y: ls.Spec.Y}, Corridor: ls.Corridor})
	}
	return out
}

// ResetForNewLap re-arms every sign so it's routed again on the next lap,
// matching reset_for_new_lap. Without this, a sign marked passed on lap 1
// stays passed for the rest of the run -- the Obstacles Challenge requires
// clearing every sign on all 3 laps.
func (r *SignRouter) ResetForNewLap() {
	r.passed = map[int]struct{}{}
	r.engaged = map[int]struct{}{}
	r.lapTick = 0
	r.committed = nil
	r.commitYaw = map[int]float64{}
	r.wrongSide = map[int]struct{}{}
}

// WrongSideViolations returns the sign indices retired as passed on the
// WRONG side of the corridor, matching the wrong_side_violations property.
// Emptied by ResetForNewLap so each lap is judged independently.
func (r *SignRouter) WrongSideViolations() map[int]struct{} {
	out := make(map[int]struct{}, len(r.wrongSide))
	for i := range r.wrongSide {
		out[i] = struct{}{}
	}
	return out
}

// DeformWaypoint returns a (possibly laterally deformed) version of the
// target waypoint, matching deform_waypoint. Checks all uncleared signs;
// the NEAREST active sign within activation distance drives the
// deformation. Camera observations are used to confirm the sign color if
// available and within match distance.
//
// waypoint is the current target waypoint; robotPos the current robot
// position; robotYawRad the robot heading (0 = east); corridor the current
// track section. Returns the deformed waypoint, unchanged if no active
// sign nearby.
func (r *SignRouter) DeformWaypoint(
	waypoint, robotPos trackmodel.Waypoint,
	robotYawRad float64,
	corridor trackmodel.Section,
	observations []TrafficSignObservation,
) trackmodel.Waypoint {
	candidates := r.preferCommitted(r.activeSignCandidates(robotPos, robotYawRad, corridor))

	// Walk candidates nearest-first and use the first whose deformation is
	// actually applicable, rather than giving up entirely if the closest
	// one is not (see deform_waypoint's Python docstring for why).
	nearestIdx := -1
	found := false
	for _, candidate := range candidates {
		if candidate.Dist > r.config.ActivationDistM {
			return waypoint
		}
		signCorridor := r.signCorridors[candidate.Index]
		if IsSquarelyInCorridor(waypoint.X, waypoint.Y, signCorridor, r.config) {
			nearestIdx = candidate.Index
			found = true
			break
		}
	}
	if !found {
		r.committed = nil
		return waypoint
	}

	if r.committed == nil || *r.committed != nearestIdx {
		r.commitYaw[nearestIdx] = robotYawRad
	}
	committedIdx := nearestIdx
	r.committed = &committedIdx
	yawDrift := math.Abs(navutil.WrapAngle(robotYawRad - r.commitYaw[nearestIdx]))

	sign := r.signs[nearestIdx]
	color := sign.Color
	if len(observations) > 0 {
		if camColor, ok := MatchDetectionToSign(
			observations, trackmodel.Waypoint{X: sign.X, Y: sign.Y}, r.config,
		); ok {
			color = camColor
		}
	}

	// Taper the offset so it fades in and out over passed_dist instead of
	// snapping between full magnitude and zero in a single waypoint step.
	// Taper on whichever of the ROBOT or the TARGET POINT is nearer the
	// sign -- see deform_waypoint's Python docstring for why keying on the
	// target alone under-delivers exactly at the moment the robot draws
	// level with the sign.
	signPos := trackmodel.Waypoint{X: sign.X, Y: sign.Y}
	influenceDist := math.Min(waypoint.DistanceTo(signPos), robotPos.DistanceTo(signPos))
	taper := math.Max(0.0, 1.0-influenceDist/r.config.PassedDistM)
	effectiveOffset := r.config.LateralOffsetM * taper

	var robotPosPtr *trackmodel.Waypoint
	if r.config.DepthPin {
		rp := robotPos
		robotPosPtr = &rp
	}
	signCorridor := r.signCorridors[nearestIdx]
	return ApplyDeformation(
		waypoint, sign, color, signCorridor, r.direction, effectiveOffset,
		PinContext{RobotPos: robotPosPtr, YawDriftRad: &yawDrift}, r.config,
	)
}

// geometricCorridor is the corner tie-break for a sign, on depth rather
// than nearest face when cfg.DepthConsistentCorridor is set, matching
// _geometric_corridor.
func (r *SignRouter) geometricCorridor(spec SignSpec) trackmodel.Section {
	corridor := waypoints.CorridorForPosition(spec.X, spec.Y, r.config.TrackCornerMinM, r.config.TrackCornerMaxM)
	if !r.config.DepthConsistentCorridor {
		return corridor
	}
	return DepthConsistentCorridor(spec.X, spec.Y, corridor, r.config)
}

// corridorForSpec is the corridor for a sign at construction time,
// preferring one whose lane target is satisfiable when
// cfg.RelabelUnsatisfiable is set, matching _corridor_for_spec.
func (r *SignRouter) corridorForSpec(spec SignSpec) trackmodel.Section {
	corridor := r.geometricCorridor(spec)
	if !r.config.RelabelUnsatisfiable {
		return corridor
	}
	return SatisfiableCorridor(spec, corridor, r.config.LateralOffsetM, r.config)
}

// recordPassSide decides whether index was cleared on its permitted side,
// matching _record_pass_side. The permitted side is absolute, fixed by the
// corridor geometry and the sign color -- red outward, green inward -- and
// is exactly the lateral direction routingTable deforms toward for that
// color. The robot's lateral coordinate relative to the sign's is compared
// against it: same sign -> correct side, opposite sign -> wrong-side pass,
// recorded in wrongSide.
func (r *SignRouter) recordPassSide(index int, robotPos trackmodel.Waypoint) {
	sign := r.signs[index]
	entry, ok := routingEntry(r.signCorridors[index], trackmodel.Clockwise)
	if !ok {
		return
	}
	permitted := multForColor(entry, sign.Color)

	robotLat, signLat := robotPos.X, sign.X
	if entry.Axis == AxisY {
		robotLat, signLat = robotPos.Y, sign.Y
	}
	side := 0
	switch {
	case robotLat > signLat:
		side = 1
	case robotLat < signLat:
		side = -1
	}
	if side != 0 && side != permitted {
		r.wrongSide[index] = struct{}{}
	}
}

// preferCommitted keeps routing around the sign already being routed
// around, matching _prefer_committed. Where two signs are both in play,
// the winner of a pure nearest-wins race can flip while the chassis is
// already committed, jumping the commanded lateral line from one sign's
// required value to the other's with no runway left to track it. So a
// sign that is still an applicable candidate holds its claim -- dropped as
// soon as it stops being reachable (retired, falls behind, leaves
// activation range, or yields no applicable deformation).
func (r *SignRouter) preferCommitted(candidates []signCandidate) []signCandidate {
	if !r.config.CommitHysteresis || r.committed == nil {
		return candidates
	}
	for _, entry := range candidates {
		if entry.Index != *r.committed {
			continue
		}
		if entry.Dist > r.config.ActivationDistM {
			break
		}
		out := make([]signCandidate, 0, len(candidates))
		out = append(out, entry)
		for _, c := range candidates {
			if c.Index != *r.committed {
				out = append(out, c)
			}
		}
		return out
	}
	r.committed = nil
	return candidates
}

// activeSignCandidates returns not-yet-passed signs near corridor, as
// (index, distance) nearest-first, matching _active_sign_candidates. Also
// maintains engagement/passed bookkeeping: a sign is engaged once the
// robot comes within activation distance, and retired only after it has
// been engaged and then left beyond passed_dist -- never discarded from
// afar. Candidates are additionally restricted to signs not already behind
// the robot (measured along its heading, with BehindToleranceM slack), and
// to signs that either belong to corridor or are within ActivationDistM of
// the robot -- see the Python docstring for why same-corridor-OR-within-
// activation_dist beats strict equality at a corner.
func (r *SignRouter) activeSignCandidates(
	robotPos trackmodel.Waypoint, robotYawRad float64, corridor trackmodel.Section,
) []signCandidate {
	r.lapTick++
	settled := r.lapTick > r.config.SettleTicks
	var candidates []signCandidate

	for i, sign := range r.signs {
		if _, isPassed := r.passed[i]; isPassed {
			continue
		}
		d := robotPos.DistanceTo(trackmodel.Waypoint{X: sign.X, Y: sign.Y})
		if settled && d < r.config.ActivationDistM {
			r.engaged[i] = struct{}{}
		}
		if d > r.config.PassedDistM {
			if _, isEngaged := r.engaged[i]; settled && isEngaged {
				r.passed[i] = struct{}{}
				r.recordPassSide(i, robotPos)
			}
			continue
		}

		// A sign the robot has already driven past needs no avoidance --
		// see _active_sign_candidates' Python docstring for why letting one
		// stay a candidate actively harms the next sign.
		dx, dy := sign.X-robotPos.X, sign.Y-robotPos.Y
		alongTrack := dx*math.Cos(robotYawRad) + dy*math.Sin(robotYawRad)
		if alongTrack < -r.config.BehindToleranceM {
			continue
		}

		sameCorridor := r.signCorridors[i] == corridor
		if !sameCorridor && d > r.config.ActivationDistM {
			continue
		}
		candidates = append(candidates, signCandidate{Index: i, Dist: d})
	}

	sort.Slice(candidates, func(a, b int) bool { return candidates[a].Dist < candidates[b].Dist })
	return candidates
}
