package collision

import (
	"math"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navutil"
)

// ContactSurface identifies which surface the chassis is touching, if any,
// matching track_model.py's ContactSurface. Kept distinct because the two
// challenges forbid different walls: the Open Challenge is scored on not
// touching the OUTER wall, the Obstacles Challenge on not touching the
// INNER one -- see (*TrackModel).ContactSurfaceAt.
type ContactSurface int

// ObstacleBox is a ground obstacle -- a traffic sign or a parking block --
// matching track_model.py's ObstacleBox. Both are short boxes standing on
// the mat, modeled the same way: an axis-aligned footprint the chassis
// can hit and the LIDAR can see. Yaw is only ever 0 or +-90 degrees for the
// objects the WRO generator emits, so a rotated block is represented by
// swapping its extents rather than carrying a general oriented box through
// the raycast maths.
type ObstacleBox struct {
	CX, CY       float64
	SizeX, SizeY float64
	// IsParkingLot reports whether this box is a lot marker fin rather than
	// a traffic sign, matching ObstacleBox.is_parking_lot. They are the same
	// shape and were therefore the same thing to this model, but the rules
	// treat them oppositely -- see ContactSurface. Defaults to false
	// (traffic sign), so every existing construction site keeps its
	// original meaning.
	IsParkingLot bool
}

// ObstacleSpec is one already-parsed obstacle pose -- the Go equivalent of
// what obstacles_from_metadata would extract from one entry of a
// scenario-metadata dict (a sign position, or a parking block's position +
// yaw). See the package doc for why this package takes parsed poses
// directly instead of a metadata-dict-parsing layer.
type ObstacleSpec struct {
	CX, CY, Length, Width, Yaw float64
	// IsParkingLot marks this spec as a lot marker fin -- see
	// ObstacleBox.IsParkingLot.
	IsParkingLot bool
}

// The surfaces a chassis footprint can be touching. SurfaceObstacle is a
// traffic sign, which may be legally nudged (WRO 9.20). SurfaceParkingLot is
// a lot marker fin, which may NOT (9.24.7) -- contact with it is an
// unconditional terminal surface, unlike a sign.
const (
	SurfaceNone ContactSurface = iota
	SurfaceOuterWall
	SurfaceInnerWall
	SurfaceObstacle
	SurfaceParkingLot
)

// noMargin is toBox's margin when the caller wants the obstacle's true
// (unmargined) footprint -- signs and parking blocks are small, rigid, and
// modeled at their true size, unlike the walls' separate fatter collision
// mesh, so visual and collision bounds are the same box.
const noMargin = 0.0

// String names each ContactSurface, matching the Python StrEnum's values.
func (s ContactSurface) String() string {
	switch s {
	case SurfaceNone:
		return "none"
	case SurfaceOuterWall:
		return "outer_wall"
	case SurfaceInnerWall:
		return "inner_wall"
	case SurfaceObstacle:
		return "obstacle"
	case SurfaceParkingLot:
		return "parking_lot"
	default:
		return "unknown"
	}
}

// NewObstacleBoxFromPose builds a box from a center pose, swapping extents
// for a quarter-turned yaw, matching ObstacleBox.from_pose.
//
// axisAlignTolerance is |cos(yaw)|'s threshold for "quarter-turned",
// matching SimulationParams.AXIS_ALIGN_TOLERANCE (collision.Config).
func NewObstacleBoxFromPose(cx, cy, length, width, yaw, axisAlignTolerance float64, isParkingLot bool) ObstacleBox {
	quarterTurned := math.Abs(math.Cos(yaw)) < axisAlignTolerance
	sizeX, sizeY := length, width
	if quarterTurned {
		sizeX, sizeY = width, length
	}
	return ObstacleBox{CX: cx, CY: cy, SizeX: sizeX, SizeY: sizeY, IsParkingLot: isParkingLot}
}

// toBox returns the axis-aligned bounds, optionally grown by margin,
// matching ObstacleBox.to_box.
func (o ObstacleBox) toBox(margin float64) box {
	halfX := o.SizeX/navutil.Half + margin
	halfY := o.SizeY/navutil.Half + margin
	return box{o.CX - halfX, o.CY - halfY, o.CX + halfX, o.CY + halfY}
}

// ObstaclesFromSpecs builds one ObstacleBox per spec, matching the
// box-construction half of obstacles_from_metadata (not its dict-parsing
// half -- see ObstacleSpec).
func ObstaclesFromSpecs(specs []ObstacleSpec, axisAlignTolerance float64) []ObstacleBox {
	boxes := make([]ObstacleBox, len(specs))
	for i, s := range specs {
		boxes[i] = NewObstacleBoxFromPose(s.CX, s.CY, s.Length, s.Width, s.Yaw, axisAlignTolerance, s.IsParkingLot)
	}
	return boxes
}
