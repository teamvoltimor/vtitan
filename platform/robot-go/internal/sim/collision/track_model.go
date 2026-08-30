package collision

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/trackmodel"
)

// NewTrackModelParams bundles NewTrackModel's inputs. Geometry and the
// track's outer-boundary coordinates come from profile.TrackConfig; the
// collision margin comes from collision.Config (simulation.toml) rather
// than a NavigationTuning field this package re-derives.
type NewTrackModelParams struct {
	Geometry trackmodel.CorridorGeometry
	// MinCoordM/MaxCoordM are the track's outer boundary, matching
	// TrackDimensions.MIN_COORD/MAX_COORD (profile.TrackConfig.Track).
	MinCoordM, MaxCoordM float64
	// Obstacles are the traffic signs and parking blocks standing on the
	// mat -- empty for the Open Challenge, which has neither.
	Obstacles []ObstacleBox
	// LidarSeesObstacles: whether obstacles occlude LIDAR rays. Both signs
	// and parking blocks are 0.10m tall -- exactly the chassis height --
	// so a deck-mounted C1 scans right at their top edge and real-world
	// detection is marginal. True models them as visible; false simulates
	// a LIDAR mounted above them, in which case only the camera perceives
	// them.
	LidarSeesObstacles bool
	// CollisionMarginM matches SimulationParams.COLLISION_MARGIN_M.
	CollisionMarginM float64
}

// TrackModel is the wall geometry + sensor/collision queries for one Open
// Challenge layout, matching track_model.py's TrackModel.
type TrackModel struct {
	walls *trackmodel.TrackWalls

	obstacleBoxes      []box
	lidarSeesObstacles bool

	innerVisual    box
	innerCollision box
	outerCollision box
}

// NewTrackModel builds the track from corridor geometry, matching
// TrackModel.__init__.
func NewTrackModel(p NewTrackModelParams) *TrackModel {
	walls := trackmodel.NewTrackWalls(p.Geometry, p.MinCoordM, p.MaxCoordM)

	// Signs and parking blocks are small, rigid and modeled at their true
	// size -- unlike the walls there is no separate fatter collision mesh,
	// so visual and collision bounds are the same box.
	obstacleBoxes := make([]box, len(p.Obstacles))
	for i, ob := range p.Obstacles {
		obstacleBoxes[i] = ob.toBox(noMargin)
	}

	inner := walls.InnerBlock
	innerVisual := box{inner.XMin, inner.YMin, inner.XMax, inner.YMax}
	m := p.CollisionMarginM
	innerCollision := box{inner.XMin - m, inner.YMin - m, inner.XMax + m, inner.YMax + m}
	// Footprint must stay within this outer collision boundary.
	outerCollision := box{p.MinCoordM + m, p.MinCoordM + m, p.MaxCoordM - m, p.MaxCoordM - m}

	return &TrackModel{
		walls:              walls,
		obstacleBoxes:      obstacleBoxes,
		lidarSeesObstacles: p.LidarSeesObstacles,
		innerVisual:        innerVisual,
		innerCollision:     innerCollision,
		outerCollision:     outerCollision,
	}
}

// Walls returns the wall geometry, for code that needs to predict scans
// from a pose -- the same object a Go LIDAR localizer would build for
// itself from scenario metadata (matches TrackModel.walls).
func (m *TrackModel) Walls() *trackmodel.TrackWalls {
	return m.walls
}

// RaycastScan casts a fan of rays and returns the nearest wall/obstacle
// range per ray, matching TrackModel.raycast_scan.
//
// x, y: sensor world position (meters). yaw: robot heading (radians).
// anglesRobot: ray bearings in the robot frame (radians, 0 = forward).
// lidarMinRangeM/maxRangeM: the sensor's floor and ceiling; a ray that
// hits nothing returns maxRangeM.
func (m *TrackModel) RaycastScan(
	x, y, yaw float64, anglesRobot []float64, lidarMinRangeM, maxRangeM float64,
) []float64 {
	ranges := m.walls.Raycast(x, y, yaw, anglesRobot, lidarMinRangeM, maxRangeM)
	if !m.lidarSeesObstacles || len(m.obstacleBoxes) == 0 {
		return ranges
	}

	// An obstacle only shortens a ray -- never lengthens it -- so fold each
	// box in with an elementwise minimum against the wall ranges.
	for i, angleRobot := range anglesRobot {
		worldAngle := yaw + angleRobot
		dx, dy := math.Cos(worldAngle), math.Sin(worldAngle)
		for _, b := range m.obstacleBoxes {
			if hit := raycastBox(x, y, dx, dy, b, maxRangeM); hit < ranges[i] {
				ranges[i] = hit
			}
		}
		ranges[i] = navutil.Clamp(ranges[i], lidarMinRangeM, maxRangeM)
	}
	return ranges
}

// FootprintCollides reports whether the oriented chassis rectangle hits a
// wall or an obstacle, matching TrackModel.footprint_collides.
func (m *TrackModel) FootprintCollides(x, y, yaw, length, width float64) bool {
	return m.ContactSurfaceAt(x, y, yaw, length, width) != SurfaceNone
}

// ContactSurfaceAt returns which surface the oriented chassis rectangle is
// touching, if any, matching TrackModel.contact_surface (renamed to avoid
// colliding with the ContactSurface type in this package's Go API).
//
// The caller needs the distinction because the two challenges forbid
// different walls: the Open Challenge is scored on not touching the OUTER
// wall, the Obstacles Challenge on not touching the INNER one. Collapsing
// all three surfaces into one boolean makes both rules unrepresentable.
//
// Checked outer first, then inner, then obstacles. The order only decides
// what a simultaneous multi-surface contact reports, which is a wedged
// robot either way.
func (m *TrackModel) ContactSurfaceAt(x, y, yaw, length, width float64) ContactSurface {
	corners := rectCorners(x, y, yaw, length, width)

	// Outer boundary: every corner must stay inside the collision box.
	ob := m.outerCollision
	for _, c := range corners {
		if c.X < ob.xMin || c.X > ob.xMax || c.Y < ob.yMin || c.Y > ob.yMax {
			return SurfaceOuterWall
		}
	}

	// Inner block: oriented footprint must not overlap the keep-out box.
	if convexOverlap(corners, m.innerCollision.corners(), yaw) {
		return SurfaceInnerWall
	}

	for _, b := range m.obstacleBoxes {
		if convexOverlap(corners, b.corners(), yaw) {
			return SurfaceObstacle
		}
	}
	return SurfaceNone
}

// ObstacleDisplacements reports how far the chassis has intruded into each
// obstacle it overlaps, matching TrackModel.obstacle_displacements. Keyed
// by obstacle index (in meters), omitting the ones not touched.
//
// This is the quantity the Obstacles Challenge is actually scored on.
// Contact with a pillar is legal -- it may be nudged, and the run stands
// as long as any corner of it is still inside its 85mm placement circle.
// Treating first contact as a crash, which is what ContactSurfaceAt alone
// supports, therefore fails runs the judges would pass. The intrusion
// depth stands in for how far the pillar was shoved.
func (m *TrackModel) ObstacleDisplacements(x, y, yaw, length, width float64) map[int]float64 {
	corners := rectCorners(x, y, yaw, length, width)
	depths := make(map[int]float64)
	for i, b := range m.obstacleBoxes {
		if depth := convexPenetration(corners, b.corners(), yaw); depth > 0.0 {
			depths[i] = depth
		}
	}
	return depths
}

// ObstacleCenter returns the center of the indexed obstacle, matching
// TrackModel.obstacle_center. The bool is false if there is no such box.
//
// Needed to tell a push from a slide: only the component of chassis travel
// pointing at a pillar displaces it.
func (m *TrackModel) ObstacleCenter(index int) (trackmodel.Waypoint, bool) {
	if index < 0 || index >= len(m.obstacleBoxes) {
		return trackmodel.Waypoint{}, false
	}
	b := m.obstacleBoxes[index]
	return trackmodel.Waypoint{X: (b.xMin + b.xMax) / navutil.Half, Y: (b.yMin + b.yMax) / navutil.Half}, true
}

// PointInFreeSpace reports whether (x, y) is in the navigable ring with
// clearance margin, matching TrackModel.point_in_free_space. Used to
// validate that planned waypoints sit inside the corridor with at least
// clearance meters to the nearest VISUAL wall.
func (m *TrackModel) PointInFreeSpace(x, y, clearance float64) bool {
	return m.walls.PointInFreeSpace(x, y, clearance)
}

// InnerBlockVisual returns the inner-block visual bounds, matching
// TrackModel.inner_block_visual.
func (m *TrackModel) InnerBlockVisual() trackmodel.InnerBlock {
	return trackmodel.InnerBlock{
		XMin: m.innerVisual.xMin, YMin: m.innerVisual.yMin,
		XMax: m.innerVisual.xMax, YMax: m.innerVisual.yMax,
	}
}
