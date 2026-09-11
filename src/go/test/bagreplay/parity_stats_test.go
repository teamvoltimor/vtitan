package bagreplay_test

import (
	"fmt"
	"math"
	"sort"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
	"github.com/teamvoltimor/vtitan/src/go/test/bagreplay"
)

// floatTol is the absolute tolerance for considering two float debug values a
// match. Debug values are reported to ~3 significant figures in the bag, so
// 1e-3 is tight enough to catch real behavioral drift without flagging
// rounding in the last place.
const floatTol = 1e-3

// fieldStat accumulates the comparison outcome for one debug field across all
// replayed ticks.
type fieldStat struct {
	name     string
	bothNil  int // neither side computed it (not a parity concern)
	goOnly   int // Go set it, Python left it null (may be a real gap or path artifact)
	refOnly  int // Python set it, Go left it null
	equal    int // both set and within tolerance
	mismatch int // both set but beyond tolerance
}

func (f *fieldStat) total() int    { return f.bothNil + f.goOnly + f.refOnly + f.equal + f.mismatch }
func (f *fieldStat) computed() int { return f.goOnly + f.refOnly + f.equal + f.mismatch }

func (f *fieldStat) record(goSet, refSet bool, within bool) {
	switch {
	case !goSet && !refSet:
		f.bothNil++
	case goSet && !refSet:
		f.goOnly++
	case !goSet && refSet:
		f.refOnly++
	case within:
		f.equal++
	default:
		f.mismatch++
	}
}

// parityStats holds the running comparison across every replayed tick.
type parityStats struct {
	// Compared float/numeric fields.
	fields map[string]*fieldStat

	// Phase comparison.
	phaseBothNil  int
	phaseEqual    int
	phaseMismatch int
	phaseMatrix   map[string]map[string]int // goPhase -> refPhase -> count
	scanTicks     int                       // ticks the Go side had a scan
	noScanTicks   int
}

func phaseKey(p string) string {
	if p == "" {
		return "<nil>"
	}
	return p
}

func newParityStats() *parityStats {
	names := []string{
		"pose_x", "pose_y", "pose_yaw",
		"waypoint_index", "laps_completed", "num_laps",
		"is_stuck", "stuck_count", "recent_movement_m",
		"forward_clearance_m", "min_lidar_range_m",
		"risk", "escape_risk",
		"crosstrack_error_m", "lookahead_distance_m", "path_turn_ahead_rad",
		"steer_target_x", "steer_target_y", "angle_error_rad",
		"clearance_speed_mps", "heading_speed_mps",
		"commanded_speed_mps", "commanded_steering_norm",
		"maneuver_steering", "maneuver_speed_mps", "maneuver_frames_left", "escape_count",
		"active_maneuver_type",
		"active_sign_count", "sign_deform_magnitude_m",
	}
	s := &parityStats{
		fields:      make(map[string]*fieldStat, len(names)),
		phaseMatrix: make(map[string]map[string]int),
	}
	for _, n := range names {
		s.fields[n] = &fieldStat{name: n}
	}
	return s
}

// compare records one tick's Go snapshot (got) against the Python reference
// (ref). Parking and BlindCreep fields are intentionally NOT compared: the Go
// port never sets them (see node/nav/debug.go and navigator/doc.go), so their
// absence is a documented, accepted gap, not a parity failure. hadScan reports
// whether the Go navigator was actually fed a /scan this tick (not a proxy
// inferred from which branch it took).
func (s *parityStats) compare(got navigator.DebugSnapshot, ref bagreplay.NavDebugSnapshot, hadScan bool) {
	// Phase.
	goPhase := got.Phase.String()
	if ref.Phase == "" {
		s.phaseBothNil++
	} else if goPhase == ref.Phase {
		s.phaseEqual++
	} else {
		s.phaseMismatch++
	}
	gp, rp := phaseKey(goPhase), phaseKey(ref.Phase)
	if s.phaseMatrix[gp] == nil {
		s.phaseMatrix[gp] = map[string]int{}
	}
	s.phaseMatrix[gp][rp]++

	// Scan coverage: the Go navigator's perception is only as good as the
	// scan it was fed this tick.
	if hadScan {
		s.scanTicks++
	} else {
		s.noScanTicks++
	}

	// Pose + race state.
	s.f("pose_x", optF64(got.PoseX), ref.PoseX)
	s.f("pose_y", optF64(got.PoseY), ref.PoseY)
	s.f("pose_yaw", optF64(got.PoseYaw), ref.PoseYaw)
	s.fInt("waypoint_index", optInt(got.WaypointIndex), ref.WaypointIndex)
	s.fInt("laps_completed", &got.LapsCompleted, &ref.LapsCompleted)
	s.fInt("num_laps", &got.NumLaps, &ref.NumLaps)

	// Stuck.
	s.fBool("is_stuck", got.IsStuck, ref.IsStuck)
	s.fInt("stuck_count", optInt(got.StuckCount), ref.StuckCount)
	s.f("recent_movement_m", optF64(got.RecentMovementM), ref.RecentMovementM)

	// Perception / risk.
	s.f("forward_clearance_m", optF64(got.ForwardClearanceM), ref.ForwardClearanceM)
	s.f("min_lidar_range_m", optF64(got.MinLidarRangeM), ref.MinLidarRangeM)
	s.fRisk("risk", got.Risk, ref.Risk)
	s.fRisk("escape_risk", got.EscapeRisk, ref.EscapeRisk)
	// rear_clearance_m: Python carries it but the Go port's perception does
	// not compute a rear channel here; skip rather than flag a known gap.

	// Path tracking.
	s.f("crosstrack_error_m", optF64(got.CrosstrackErrorM), ref.CrosstrackErrorM)
	s.f("lookahead_distance_m", optF64(got.LookaheadDistance), ref.LookaheadDistanceM)
	s.f("path_turn_ahead_rad", optF64(got.PathTurnAheadRad), ref.PathTurnAheadRad)
	s.f("steer_target_x", optF64(got.SteerTargetX), ref.SteerTargetX)
	s.f("steer_target_y", optF64(got.SteerTargetY), ref.SteerTargetY)
	s.f("angle_error_rad", optF64(got.AngleErrorRad), ref.AngleErrorRad)

	// Speed selection.
	s.f("clearance_speed_mps", optF64(got.ClearanceSpeedMPS), ref.ClearanceSpeedMPS)
	s.f("heading_speed_mps", optF64(got.HeadingSpeedMPS), ref.HeadingSpeedMPS)

	// Final command.
	s.f("commanded_speed_mps", optF64(got.CommandedSpeedMPS), ref.CommandedSpeedMPS)
	s.f("commanded_steering_norm", optF64(got.CommandedSteerNorm), ref.CommandedSteeringNorm)

	// Maneuver.
	s.fManeuver("active_maneuver_type", got.ActiveManeuverType, ref.ActiveManeuverType)
	s.f("maneuver_steering", optF64(got.ManeuverSteering), ref.ManeuverSteering)
	s.f("maneuver_speed_mps", optF64(got.ManeuverSpeedMPS), ref.ManeuverSpeedMPS)
	s.fInt("maneuver_frames_left", optInt(got.ManeuverFramesLeft), ref.ManeuverFramesLeft)
	s.fInt("escape_count", optInt(got.EscapeCount), ref.EscapeCount)

	// Sign routing.
	s.fInt("active_sign_count", optInt(got.ActiveSignCount), ref.ActiveSignCount)
	s.f("sign_deform_magnitude_m", optF64(got.SignDeformMagnitudeM), ref.SignDeformMagnitudeM)
}

func (s *parityStats) f(name string, goV *float64, refV *float64) {
	fs := s.fields[name]
	goSet, refSet := goV != nil, refV != nil
	within := false
	if goSet && refSet {
		within = math.Abs(*goV-*refV) <= floatTol
	}
	fs.record(goSet, refSet, within)
}

func (s *parityStats) fInt(name string, goV *int, refV *int) {
	fs := s.fields[name]
	goSet, refSet := goV != nil, refV != nil
	within := false
	if goSet && refSet {
		within = *goV == *refV
	}
	fs.record(goSet, refSet, within)
}

func (s *parityStats) fBool(name string, goV *bool, refV *bool) {
	fs := s.fields[name]
	goSet, refSet := goV != nil, refV != nil
	within := false
	if goSet && refSet {
		within = *goV == *refV
	}
	fs.record(goSet, refSet, within)
}

func (s *parityStats) fRisk(name string, goV *controllers.RiskLevel, refV *string) {
	fs := s.fields[name]
	goSet := goV != nil
	// Python writes risk as a string token; map it onto the Go enum to compare.
	var refEnum *controllers.RiskLevel
	if refV != nil {
		e := riskFromToken(*refV)
		refEnum = &e
	}
	refSet := refEnum != nil
	within := false
	if goSet && refSet {
		within = *goV == *refEnum
	}
	fs.record(goSet, refSet, within)
}

func (s *parityStats) fManeuver(name string, goV *controllers.ManeuverType, refV *string) {
	fs := s.fields[name]
	goSet := goV != nil
	var refEnum *controllers.ManeuverType
	if refV != nil {
		e := maneuverFromToken(*refV)
		if e != nil {
			refEnum = e
		}
	}
	refSet := refEnum != nil
	within := false
	if goSet && refSet {
		within = *goV == *refEnum
	}
	fs.record(goSet, refSet, within)
}

func (s *parityStats) report() []string {
	var out []string
	out = append(out, fmt.Sprintf("SCAN COVERAGE  hadScan=%d noScan=%d", s.scanTicks, s.noScanTicks))
	out = append(out, fmt.Sprintf("PHASE  equal=%d mismatch=%d bothNil=%d",
		s.phaseEqual, s.phaseMismatch, s.phaseBothNil))
	out = append(out, "PHASE CONFUSION (goPhase -> refPhase: count), top entries:")
	goPhases := sortedKeys(s.phaseMatrix)
	for _, gp := range goPhases {
		inner := s.phaseMatrix[gp]
		refPhases := sortedKeys(inner)
		parts := make([]string, 0, len(refPhases))
		for _, rp := range refPhases {
			parts = append(parts, fmt.Sprintf("%s=%d", rp, inner[rp]))
		}
		out = append(out, fmt.Sprintf("  %-18s -> %v", gp, parts))
	}
	out = append(out, "FIELD                          bothNil goOnly refOnly  equal mismatch  match%%")
	for _, name := range sortedFieldNames(s.fields) {
		fs := s.fields[name]
		computed := fs.computed()
		pct := 0.0
		if computed > 0 {
			pct = 100.0 * float64(fs.equal) / float64(computed)
		}
		out = append(out, fmt.Sprintf("%-28s %7d %6d %7d %6d %8d   %5.1f",
			name, fs.bothNil, fs.goOnly, fs.refOnly, fs.equal, fs.mismatch, pct))
	}
	return out
}

func sortedKeys[V any](m map[string]V) []string {
	keys := make([]string, 0, len(m))
	for k := range m {
		keys = append(keys, k)
	}
	sort.Strings(keys)
	return keys
}

// reportPerception logs the perception-group match rates. These depend only on
// scan+pose (not the reconstructed path), so they are the cleanest parity
// signal; a low rate points at a real scan-decode or perception difference
// rather than a path artifact.
func (s *parityStats) reportPerception(t *testing.T) {
	t.Helper()
	for _, name := range []string{"min_lidar_range_m", "forward_clearance_m", "risk"} {
		fs := s.fields[name]
		computed := fs.computed()
		if computed == 0 {
			t.Logf("perception %s: no ticks where both sides set it", name)
			continue
		}
		pct := 100.0 * float64(fs.equal) / float64(computed)
		t.Logf("perception %s match=%.1f%% (%d/%d computed)", name, pct, fs.equal, computed)
	}
}

// --- token/enum mapping helpers ---

func riskFromToken(s string) controllers.RiskLevel {
	switch s {
	case "safe":
		return controllers.RiskSafe
	case "obstacle":
		return controllers.RiskObstacle
	case "critical":
		return controllers.RiskCritical
	default:
		return controllers.RiskSafe
	}
}

func maneuverFromToken(s string) *controllers.ManeuverType {
	var m controllers.ManeuverType
	switch s {
	case "k_turn":
		m = controllers.ManeuverKTurn
	case "side_correction":
		m = controllers.ManeuverSideCorrection
	case "stuck_reverse":
		m = controllers.ManeuverStuckReverse
	case "stuck_forward":
		m = controllers.ManeuverStuckForward
	default:
		return nil
	}
	return &m
}

func optF64(v *float64) *float64 { return v }
func optInt[T any](v *T) *T      { return v }

var _ = trackmodel.Clockwise

func sortedFieldNames(m map[string]*fieldStat) []string {
	names := make([]string, 0, len(m))
	for n := range m {
		names = append(names, n)
	}
	sort.Strings(names)
	return names
}
