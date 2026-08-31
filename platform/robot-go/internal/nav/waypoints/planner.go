package waypoints

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// StartingConditions is the robot's believed start pose and travel direction,
// matching shared.domain.models.StartingConditions (the fields
// calculate_waypoints / plan_believed_path actually read). Direction is
// Optional: a nil means the direction has not been resolved yet, and
// CalculateWaypoints returns an error rather than silently planning a
// reflected path (Python raises ValueError for the same reason).
type StartingConditions struct {
	Direction *trackmodel.Direction
	Section   trackmodel.Section
	Position  trackmodel.Waypoint
	Yaw       float64
}

// ReplannedAt returns a copy of sc with the believed pose/direction swapped
// in, matching StartingConditions.replanned_at -- typed replacement for a
// believed pose during replanning.
func (sc StartingConditions) ReplannedAt(
	direction *trackmodel.Direction,
	section trackmodel.Section,
	position trackmodel.Waypoint,
	yaw float64,
) StartingConditions {
	return StartingConditions{
		Direction: direction,
		Section:   section,
		Position:  position,
		Yaw:       yaw,
	}
}

// PlannerInput is everything CalculateWaypoints needs from a scenario's
// metadata: the believed corridor geometry plus the (believed) starting
// conditions. It is the Go stand-in for the Pydantic ScenarioMetadata, which
// has no Go equivalent in this tree -- only the two fields generation.py
// actually reads are represented here, so the planner stays decoupled from the
// rest of the scenario-model layer.
type PlannerInput struct {
	Geometry  trackmodel.CorridorGeometry
	Starting  StartingConditions
	MaxCoordM float64
	// ChassisWidthM is RobotSpecs.WIDTH, used by ValidatePathFeasibility.
	ChassisWidthM float64
}

// CalculateWaypoints builds the full multi-lap waypoint sequence for a
// scenario, matching generation.py's calculate_waypoints.
//
// It reads corridor widths and starting conditions from input, constructs arc
// waypoints at corners plus straight waypoints along each corridor
// centerline, and returns the ordered list of world-frame Waypoints starting
// near the robot's spawn position, covering numLaps full loops.
//
// ArcRadius and CornerArcAssumeWide come from cfg (loaded from
// waypoints.toml); centerBiasM, when non-nil, overrides the tuning default
// uniformly with the wide side (the Obstacles Challenge passes 0.0 to plan
// down the centre) -- nil keeps cfg's per-width split.
//
// Returns an error if the starting Direction is unresolved or if the chassis
// does not fit the narrowest corridor once biased off centre.
func CalculateWaypoints(
	input PlannerInput,
	numLaps int,
	cfg Config,
	centerBiasM *float64,
) ([]trackmodel.Waypoint, error) {
	if input.Starting.Direction == nil {
		return nil, fmt.Errorf(
			"waypoints: calculate_waypoints requires a resolved StartingConditions.Direction; " +
				"callers must infer/assign it before planning a path",
		)
	}
	direction := *input.Starting.Direction

	widths := input.Geometry.ToWidthsDict()
	minWidthM := input.Geometry.MinWidthM()

	// Score feasibility with the narrowest corridor's own bias -- the wide
	// value would overstate what the narrow corridor actually spends.
	feasibility := ValidatePathFeasibility(
		minWidthM,
		CenterBiasForCorridor(minWidthM, cfg, centerBiasM),
		input.ChassisWidthM,
	)
	if !feasibility.IsFeasible {
		return nil, fmt.Errorf("waypoints: %s", feasibility.Reason)
	}

	northWidth := widths[trackmodel.North]
	southWidth := widths[trackmodel.South]
	eastWidth := widths[trackmodel.East]
	westWidth := widths[trackmodel.West]

	// Each corridor takes the bias for ITS OWN width; an explicit magnitude
	// still overrides both uniformly.
	northBias := CenterBiasForCorridor(northWidth, cfg, centerBiasM)
	southBias := CenterBiasForCorridor(southWidth, cfg, centerBiasM)
	eastBias := CenterBiasForCorridor(eastWidth, cfg, centerBiasM)
	westBias := CenterBiasForCorridor(westWidth, cfg, centerBiasM)

	// Signs put the bias toward the inner block on every side: north and east
	// corridors have the block below/left of them, south and west above/right.
	northCY := input.MaxCoordM - northWidth/2 - northBias
	southCY := southWidth/2 + southBias
	eastCX := input.MaxCoordM - eastWidth/2 - eastBias
	westCX := westWidth/2 + westBias

	// Each corner is sized by the two corridors it joins. The bias passed is
	// the WIDER corridor's, because CornerArcRadius is tangent to that
	// corridor's centerline (it takes max(entry, exit)) and the radius has to
	// preserve the clearance THAT straight has.
	assumeWide := cfg.CornerArcAssumeWide
	cornerRadii := CornerRadii{
		SE: cornerRadius(eastWidth, southWidth, cfg, assumeWide),
		SW: cornerRadius(southWidth, westWidth, cfg, assumeWide),
		NW: cornerRadius(westWidth, northWidth, cfg, assumeWide),
		NE: cornerRadius(northWidth, eastWidth, cfg, assumeWide),
	}

	segments := BuildAllSegments(
		northCY, southCY, eastCX, westCX,
		cornerRadii,
		direction,
		cfg,
	)

	order := trackmodel.LoopOrder(input.Starting.Section, direction)
	fullLoop := AssembleLoop(order, segments)

	waypoints := BuildWaypointSequence(
		fullLoop,
		segments,
		order,
		input.Starting.Position.X,
		input.Starting.Position.Y,
		numLaps,
		cfg,
	)

	if err := validateBounds(waypoints, input.MaxCoordM); err != nil {
		return nil, err
	}
	return waypoints, nil
}

// wideCorridorWidthM mirrors shared.config.constants.CorridorDimensions.WIDE
// (1.0 m) -- the fixed WRO rule width CORNER_ARC_ASSUME_WIDE substitutes for
// both corridors' believed widths, so the arc is independent of a belief that
// starts out wrong. The rules only present NARROW (0.6) and WIDE (1.0), so
// this is a constant, not config.
const wideCorridorWidthM = 1.0

// cornerRadius sizes one corner arc from the two corridors it joins, matching
// generation.py's corner_radii dict comprehension. When assumeWide is set, both
// corridors are treated as WIDE (wideCorridorWidthM), so the arc -- and
// therefore the turn-entry point -- is independent of a belief that starts out
// wrong.
func cornerRadius(
	entryW, exitW float64,
	cfg Config,
	assumeWide bool,
) float64 {
	if assumeWide {
		entryW, exitW = wideCorridorWidthM, wideCorridorWidthM
	}
	return CornerArcRadius(
		entryW,
		exitW,
		CenterBiasForCorridor(math.Max(entryW, exitW), cfg, nil),
		cfg.ArcRadius,
	)
}

// PlanBelievedPath builds a one-lap path for the layout the robot currently
// believes it is on, matching generation.py's plan_believed_path. Shared by
// the simulator and the navigator: both replan from a believed corridor-width
// estimate and a believed start pose that can differ from the ground-truth
// record, which is why the believed section/position/yaw/direction are
// threaded in separately.
//
// numLaps is forced to 1: a caller that wants the real lap count cycles this
// single canonical lap the required number of times (matching the Python
// original, where baking num_laps into the returned list would multiply laps
// when the navigator already wraps one lap).
func PlanBelievedPath(
	base PlannerInput,
	believed trackmodel.CorridorGeometry,
	direction *trackmodel.Direction,
	believedSection trackmodel.Section,
	believedPosition trackmodel.Waypoint,
	believedYaw float64,
	cfg Config,
	centerBiasM *float64,
) ([]trackmodel.Waypoint, error) {
	replanned := base
	replanned.Geometry = believed
	replanned.Starting = base.Starting.ReplannedAt(
		direction, believedSection, believedPosition, believedYaw,
	)
	return CalculateWaypoints(replanned, 1, cfg, centerBiasM)
}

// validateBounds reports an error if generation produced an out-of-bounds
// waypoint, matching generation.py's validate_bounds -- every waypoint must
// stay on the track and clear of the restricted inner square.
func validateBounds(waypoints []trackmodel.Waypoint, maxCoordM float64) error {
	const minCoordM = 0.0
	block := innerSquareExclusion(maxCoordM)
	for _, w := range waypoints {
		if w.X < minCoordM || w.X > maxCoordM || w.Y < minCoordM || w.Y > maxCoordM {
			return fmt.Errorf(
				"waypoints: generated waypoint (%.3f, %.3f) falls outside the track bounds", w.X, w.Y,
			)
		}
		if block != nil &&
			block.XMin < w.X && w.X < block.XMax && block.YMin < w.Y && w.Y < block.YMax {
			return fmt.Errorf(
				"waypoints: generated waypoint (%.3f, %.3f) falls inside the restricted inner square", w.X, w.Y,
			)
		}
	}
	return nil
}

// innerSquareExclusion returns the restricted inner-square region (the area
// the path must never enter), derived from the inner block of a
// CorridorGeometry whose corridors are all zero-width -- i.e. the pure block
// with no corridor margin. The Python original excludes the block expanded by
// nothing; here we exclude exactly the block faces (CornerMin..CornerMax),
// matching validate_bounds' intent.
func innerSquareExclusion(maxCoordM float64) *trackmodel.InnerBlock {
	// The inner block is maximized when all corridors are zero-width, which
	// yields the full [0, maxCoordM]x[0, maxCoordM] square -- the path is
	// everywhere outside it in any real layout, so a non-nil block with the
	// true corner bounds is the correct exclusion. Callers pass the real
	// MaxCoordM; the block half-extents come from the WRO rule that the inner
	// square is the central 1.0x1.0 m region on the 3.0x3.0 m mat. For a
	// generic maxCoordM we derive the square as the central region between the
	// corridor-min and corridor-max, which for the standard mat is
	// [1, 2]x[1, 2]. We reproduce that exactly: cornerMin = maxCoordM/3,
	// cornerMax = 2*maxCoordM/3, the WRO 2026 inner-square definition.
	cornerMin := maxCoordM / 3.0
	cornerMax := 2.0 * maxCoordM / 3.0
	return &trackmodel.InnerBlock{
		XMin: cornerMin,
		YMin: cornerMin,
		XMax: cornerMax,
		YMax: cornerMax,
	}
}
