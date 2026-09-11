package controllers

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"
)

// EscapeManeuver is an escape maneuver command, matching
// collision_avoidance_controller.EscapeManeuver.
type EscapeManeuver struct {
	// ManeuverType is the kind of escape (see ManeuverType).
	Type ManeuverType
	// Steering is the commanded steering angle, [-1, 1].
	Steering float64
	// Speed is the commanded speed, [-1, 1].
	Speed float64
	// DurationFrames is how long to execute this maneuver.
	DurationFrames int
	// Priority is the priority level (higher = more urgent).
	Priority int
}

// ParkingGate is the two clearance numbers the parking maneuver's
// stop-check needs, matching collision_avoidance_controller.ParkingGate.
//
// The parking controller drives laterally into a bay, so a single forward
// cone is not enough: it must also be stopped before any sideways or
// slightly-rearward clip. ForwardM is the min over the narrow forward
// cone, SweepM the min over the entire 360deg sweep.
type ParkingGate struct {
	ForwardM float64
	SweepM   float64
}

// CollisionAvoidanceController manages collision detection and escape
// maneuver generation, matching
// collision_avoidance_controller.CollisionAvoidanceController.
//
// Construction is via CollisionAvoidanceControllerFromConfig (this
// package's from_tuning equivalent), which is the single source of truth
// for these values -- constructing the struct literal directly is for
// tests that need to pin a specific value.
type CollisionAvoidanceController struct {
	// ContactDist is the critical distance threshold (m).
	ContactDist float64
	// RiskRayWindow is how many ADJACENT lane rays must corroborate a short
	// reading before it counts as an obstacle. See RobustMinRange; 1 is the
	// bare minimum this used before 2026-09-06.
	RiskRayWindow int
	// SlowDist is the begin-avoiding distance (m).
	SlowDist float64
	// FastDist is the normal-speed distance (m); unused by this
	// controller's own methods but mirrored for from_tuning parity.
	FastDist float64
	// EscapeRevSpeed is the reverse speed during escapes.
	EscapeRevSpeed float64
	// EscapeSteerScale is the steering aggressiveness during a K-turn.
	EscapeSteerScale float64
	// StuckThreshold is the distance threshold for stuck detection (m);
	// mirrored for from_tuning parity, not read by this controller's own
	// methods (see StuckDetector).
	StuckThreshold float64
	// PathHalfWidth is the half-width of the forward driving lane risk is
	// judged over -- the chassis half-width plus a clearance margin (m).
	PathHalfWidth float64
	// KTurnMinFrames/KTurnMaxFrames are the K-turn duration for
	// OBSTACLE/CRITICAL risk (frames).
	KTurnMinFrames, KTurnMaxFrames int
	// SideCorrectionSteer/SideCorrectionSpeed/SideCorrectionFrames
	// parameterize a side-threat correction.
	SideCorrectionSteer  float64
	SideCorrectionSpeed  float64
	SideCorrectionFrames int
	// FrontHalfFovRad is the half-width of the forward clearance cone
	// (radians), used by ComputeForwardClearance.
	FrontHalfFovRad float64
	// ThreatHalfFovRad is the half-width of the threat-detection sectors
	// (radians), used by DetectThreatDirection/ComputeRearClearance/etc.
	ThreatHalfFovRad float64
	// Geometry bundles the sector-math parameters (self-detection
	// threshold, min valid range, no-data sentinel, LIDAR max range,
	// blind wedges) shared by every sector query this controller makes.
	Geometry SectorGeometry
	// ThreatNoDetectionRangeM: a sector's nearest reading beyond this
	// distance doesn't count as a threat at all.
	ThreatNoDetectionRangeM float64
	// LidarToFrontBumperM is the sensor-to-front-bumper offset (m), used
	// to convert a raw forward range into a bumper gap (see BumperGapAhead).
	LidarToFrontBumperM float64
}

// AssessRisk assesses collision risk from obstacles in the robot's forward
// path, matching CollisionAvoidanceController.assess_risk.
//
// Risk is judged over the forward driving lane only (see
// ForwardPathRanges), not the full 360deg sweep: a corridor's side walls
// are not obstacles the robot is about to hit.
func (c *CollisionAvoidanceController) AssessRisk(rangesM, anglesRad []float64) RiskLevel {
	if len(rangesM) == 0 {
		return RiskSafe
	}

	path := ForwardPathRanges(
		rangesM,
		anglesRad,
		c.PathHalfWidth,
		c.Geometry.MinValidRangeM,
		c.Geometry.LidarMaxRangeM,
	)
	if len(path) == 0 {
		// Two different causes read the same here: no scan rays fell
		// inside the forward lane at all (safe by construction), or the
		// lane DID have rays but every one was a no-return -- the
		// signature of something very close spanning the WHOLE cone, not
		// of open road. Only the first case is actually safe.
		if ForwardPathHasRays(rangesM, anglesRad, c.PathHalfWidth) {
			return RiskCritical
		}
		return RiskSafe
	}

	// Corroborated by adjacent rays rather than the bare minimum: see
	// RobustMinRange for why a noisy sweep makes the raw minimum a phantom.
	minPath := RobustMinRange(path, c.RiskRayWindow)
	gap := BumperGapAhead(minPath, c.LidarToFrontBumperM)

	if gap < c.ContactDist {
		return RiskCritical
	}
	if gap < c.SlowDist {
		return RiskObstacle
	}
	return RiskSafe
}

// Sector is this controller's configured view of one angular sector,
// matching CollisionAvoidanceController.sector. The single place the
// instance's sector parameters are wired to SectorToModel, so a change to
// the mount geometry or wedge angles lands in one place.
func (c *CollisionAvoidanceController) Sector(
	rangesM, anglesRad []float64, centerRad float64, halfFovRad *float64, filterSelfDetection bool,
) SectorRanges {
	fov := c.ThreatHalfFovRad
	if halfFovRad != nil {
		fov = *halfFovRad
	}
	return SectorToModel(rangesM, anglesRad, centerRad, fov, filterSelfDetection, c.Geometry)
}

// RearSector is the rear +/-ThreatHalfFovRad sector, self-detection
// filtered, matching CollisionAvoidanceController.rear_sector. Callers
// gating a reverse want this rather than ComputeRearClearance: Measured()
// is what tells "genuinely open" apart from "this bearing is a blind spot".
func (c *CollisionAvoidanceController) RearSector(rangesM, anglesRad []float64) SectorRanges {
	return c.Sector(rangesM, anglesRad, math.Pi, nil, true)
}

// FrontSector is the forward +/-FrontHalfFovRad sector, NOT self-detection
// filtered (matching ComputeForwardClearance's own Sector call), matching
// CollisionAvoidanceController.front_sector. Callers gating on whether the
// forward cone was genuinely MEASURED (as opposed to merely reporting the
// no-data sentinel, which reads identically to open road) want this rather
// than ComputeForwardClearance -- see Measured().
func (c *CollisionAvoidanceController) FrontSector(rangesM, anglesRad []float64) SectorRanges {
	return c.Sector(rangesM, anglesRad, 0.0, &c.FrontHalfFovRad, false)
}

// ComputeForwardClearance is the minimum clearance in the forward
// FrontHalfFovRad sector (0 rad = forward), matching
// CollisionAvoidanceController.compute_forward_clearance.
func (c *CollisionAvoidanceController) ComputeForwardClearance(
	rangesM, anglesRad []float64,
) float64 {
	if len(rangesM) == 0 {
		return c.Geometry.NoDataRangeM
	}
	fov := c.FrontHalfFovRad
	sr := c.Sector(rangesM, anglesRad, 0.0, &fov, false)
	if sr.Measured() {
		return sr.MinRangeM
	}
	return c.Geometry.NoDataRangeM
}

// ComputeRearClearance is the minimum clearance in the rear sector,
// matching CollisionAvoidanceController.compute_rear_clearance. Reports
// NoDataRangeM when the rear sector saw nothing, which reads identically
// to open road -- use RearSector and check Measured() when the answer
// authorizes a reverse.
func (c *CollisionAvoidanceController) ComputeRearClearance(rangesM, anglesRad []float64) float64 {
	if len(rangesM) == 0 {
		return c.Geometry.NoDataRangeM
	}
	sr := c.RearSector(rangesM, anglesRad)
	if sr.Measured() {
		return sr.MinRangeM
	}
	return c.Geometry.NoDataRangeM
}

// ComputeMinClearance is the minimum clearance over a wide sector,
// matching CollisionAvoidanceController.compute_min_clearance. Used to
// gate maneuvers where the robot's path isn't a straight line, so a
// lateral clip is caught before it happens.
func (c *CollisionAvoidanceController) ComputeMinClearance(
	rangesM, anglesRad []float64, centerRad, halfFovRad float64,
) float64 {
	if len(rangesM) == 0 {
		return c.Geometry.NoDataRangeM
	}
	sr := c.Sector(rangesM, anglesRad, centerRad, &halfFovRad, false)
	if sr.Measured() {
		return sr.MinRangeM
	}
	return c.Geometry.NoDataRangeM
}

// ParkingClearances returns forward and full-sweep clearances for the
// parking stop-check, matching
// CollisionAvoidanceController.parking_clearances.
func (c *CollisionAvoidanceController) ParkingClearances(rangesM, anglesRad []float64) ParkingGate {
	return ParkingGate{
		ForwardM: c.ComputeForwardClearance(rangesM, anglesRad),
		SweepM:   c.ComputeMinClearance(rangesM, anglesRad, 0.0, math.Pi),
	}
}

// DetectThreatDirection is the direction of the closest obstacle, matching
// CollisionAvoidanceController.detect_threat_direction. Sectors are
// angular cones (+/-45deg default) about forward (0), left (+pi/2), right
// (-pi/2) and rear (+/-pi), so the result is correct regardless of the
// scan's index ordering.
func (c *CollisionAvoidanceController) DetectThreatDirection(
	rangesM, anglesRad []float64,
) ThreatDirection {
	if len(rangesM) == 0 {
		return ThreatNone
	}

	sectorMin := func(centerRad float64, filterSelfDetection bool) float64 {
		sr := SectorToModel(
			rangesM,
			anglesRad,
			centerRad,
			c.ThreatHalfFovRad,
			filterSelfDetection,
			c.Geometry,
		)
		if sr.ValidCount > 0 {
			return sr.MinRangeM
		}
		return c.Geometry.NoDataRangeM
	}

	// Forward is never self-detection filtered: a genuine near-contact
	// dead ahead must still register even inside that radius.
	front := sectorMin(0.0, false)
	left := sectorMin(math.Pi/2, true)
	right := sectorMin(-math.Pi/2, true)
	back := sectorMin(math.Pi, true)

	closest, closestDist := ThreatFront, front
	for _, cand := range []struct {
		dir  ThreatDirection
		dist float64
	}{{ThreatLeft, left}, {ThreatRight, right}, {ThreatBack, back}} {
		if cand.dist < closestDist {
			closest, closestDist = cand.dir, cand.dist
		}
	}

	if closestDist > c.ThreatNoDetectionRangeM {
		return ThreatNone
	}
	return closest
}

// SectorRangeValues is this controller's configured, instance-bound
// sector-range query, matching
// CollisionAvoidanceController.sector_ranges' role as the wrapper external
// callers (e.g. clearances.ClearancesFromScan) use instead of repeating
// this controller's tuning-sourced parameters themselves.
func (c *CollisionAvoidanceController) SectorRangeValues(
	rangesM, anglesRad []float64, centerRad, halfFovRad float64, filterSelfDetection bool,
) []float64 {
	return SectorRangeValues(
		rangesM,
		anglesRad,
		centerRad,
		halfFovRad,
		filterSelfDetection,
		c.Geometry,
		true,
	)
}

// ComputeEscapeManeuver generates an escape maneuver for a detected
// threat, matching CollisionAvoidanceController.compute_escape_maneuver.
// direction is the inferred travel direction (nil when undetermined), the
// fallback for the K-turn's side when LIDAR alone cannot tell (see
// kTurnSteerSign). Returns ok=false when no maneuver is needed.
func (c *CollisionAvoidanceController) ComputeEscapeManeuver(
	risk RiskLevel,
	threatDir ThreatDirection,
	rangesM, anglesRad []float64,
	direction *trackmodel.Direction,
) (maneuver EscapeManeuver, ok bool) {
	if risk == RiskSafe {
		return EscapeManeuver{}, false
	}

	switch threatDir {
	case ThreatFront:
		steerSign := c.kTurnSteerSign(rangesM, anglesRad, direction)
		steering := 0.0
		duration := c.KTurnMinFrames
		priority := 1
		if risk == RiskCritical {
			steering = c.EscapeSteerScale * steerSign
			duration = c.KTurnMaxFrames
			priority = 2
		}
		return EscapeManeuver{
			Type: ManeuverKTurn, Steering: steering, Speed: c.EscapeRevSpeed,
			DurationFrames: duration, Priority: priority,
		}, true

	case ThreatLeft:
		alreadyTouching := c.sideClearance(math.Pi/2, rangesM, anglesRad) < c.ContactDist ||
			c.forwardTouching(rangesM, anglesRad)
		steerSign := -1.0
		if alreadyTouching {
			steerSign = 1.0
		}
		return c.sideCorrectionManeuver(steerSign, alreadyTouching), true

	case ThreatRight:
		alreadyTouching := c.sideClearance(-math.Pi/2, rangesM, anglesRad) < c.ContactDist ||
			c.forwardTouching(rangesM, anglesRad)
		steerSign := 1.0
		if alreadyTouching {
			steerSign = -1.0
		}
		return c.sideCorrectionManeuver(steerSign, alreadyTouching), true

	default:
		return EscapeManeuver{}, false
	}
}

// kTurnSteerSign is the steering sign that swings the nose toward the
// clearer side in reverse, matching
// CollisionAvoidanceController._k_turn_steer_sign.
//
// Ackermann reverse flips the yaw response relative to forward travel, so
// a positive steering command swings the nose toward the robot's RIGHT
// while reversing. direction is the inferred travel direction (nil when
// undetermined), the fallback when LIDAR alone cannot tell: clockwise
// keeps the island on the robot's right for the whole lap, so the side
// away from it is the structurally safer default.
func (c *CollisionAvoidanceController) kTurnSteerSign(
	rangesM, anglesRad []float64, direction *trackmodel.Direction,
) float64 {
	if leftClear, rightClear, ok := c.leftRightClearance(rangesM, anglesRad); ok &&
		leftClear != rightClear {
		// Swing left (negative) when the left is clearer; swing right
		// (positive) when the right is clearer.
		if leftClear > rightClear {
			return -1.0
		}
		return 1.0
	}
	if direction != nil && *direction == trackmodel.Clockwise {
		return -1.0
	}
	return 1.0
}

// leftRightClearance returns the left/right sector clearances kTurnSteerSign
// compares, and whether either sector actually saw a return -- rangesM==nil
// or two no-data sectors both report ok=false so the caller falls through to
// the direction-based default instead of comparing two meaningless
// NoDataRangeM values.
func (c *CollisionAvoidanceController) leftRightClearance(
	rangesM, anglesRad []float64,
) (leftClear, rightClear float64, ok bool) {
	if rangesM == nil {
		return 0, 0, false
	}
	left := SectorToModel(rangesM, anglesRad, math.Pi/2, c.ThreatHalfFovRad, true, c.Geometry)
	right := SectorToModel(rangesM, anglesRad, -math.Pi/2, c.ThreatHalfFovRad, true, c.Geometry)
	if left.ValidCount == 0 && right.ValidCount == 0 {
		return 0, 0, false
	}
	leftClear, rightClear = c.Geometry.NoDataRangeM, c.Geometry.NoDataRangeM
	if left.ValidCount > 0 {
		leftClear = left.MinRangeM
	}
	if right.ValidCount > 0 {
		rightClear = right.MinRangeM
	}
	return leftClear, rightClear, true
}

// sideCorrectionManeuver builds the SIDE_CORRECTION EscapeManeuver shared
// by the LEFT/RIGHT branches of ComputeEscapeManeuver, which are
// identical apart from the steering sign and already-touching test.
func (c *CollisionAvoidanceController) sideCorrectionManeuver(
	steerSign float64,
	alreadyTouching bool,
) EscapeManeuver {
	speed := c.SideCorrectionSpeed
	duration := c.SideCorrectionFrames
	if alreadyTouching {
		speed = c.EscapeRevSpeed
		duration = c.KTurnMinFrames
	}
	return EscapeManeuver{
		Type: ManeuverSideCorrection, Steering: steerSign * c.SideCorrectionSteer, Speed: speed,
		DurationFrames: duration, Priority: 1,
	}
}

// sideClearance is the minimum clearance in a +/-ThreatHalfFovRad sector
// about centerRad, matching CollisionAvoidanceController._side_clearance.
// Used to tell "obstacle getting close" from "chassis is already at the
// wall" for a side threat.
func (c *CollisionAvoidanceController) sideClearance(
	centerRad float64,
	rangesM, anglesRad []float64,
) float64 {
	if rangesM == nil {
		return c.Geometry.NoDataRangeM
	}
	sr := SectorToModel(rangesM, anglesRad, centerRad, c.ThreatHalfFovRad, true, c.Geometry)
	if sr.ValidCount > 0 {
		return sr.MinRangeM
	}
	return c.Geometry.NoDataRangeM
}

// forwardTouching reports whether the chassis is already at ContactDist
// or closer straight ahead, matching
// CollisionAvoidanceController._forward_touching. Reuses AssessRisk's own
// forward-path geometry (the chassis-width lane) so this agrees with
// whatever risk level triggered the escape in the first place.
func (c *CollisionAvoidanceController) forwardTouching(rangesM, anglesRad []float64) bool {
	if len(rangesM) == 0 {
		return false
	}
	path := ForwardPathRanges(
		rangesM,
		anglesRad,
		c.PathHalfWidth,
		c.Geometry.MinValidRangeM,
		c.Geometry.LidarMaxRangeM,
	)
	if len(path) == 0 {
		return false
	}
	minPath := path[0]
	for _, r := range path[1:] {
		minPath = min(minPath, r)
	}
	return BumperGapAhead(minPath, c.LidarToFrontBumperM) < c.ContactDist
}
