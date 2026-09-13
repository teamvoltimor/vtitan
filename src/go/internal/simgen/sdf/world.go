package sdf

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// wallSpec is one exterior wall model: its name, center point and the
// visual/collision box dimensions (visual length includes full wall
// thickness; collision adds the extra collision margin).
type wallSpec struct {
	name       string
	cx, cy     float64
	visX, visY float64
	colX, colY float64
}

// gridLine is one thin grid-line model and its fixed coordinate along the
// line's normal axis.
type gridLine struct {
	name  string
	coord float64
}

// subdivisionSpec is one corridor's subdivision line placement: the
// corridor plus its center-line coordinate and the two width-marker
// coordinates.
type subdivisionSpec struct {
	section         simconfig.Section
	mid, div1, div2 float64
}

// GenerateBaseWorld returns the SDF root and world nodes for the WRO 2026 track.
// The world includes physics, lights, ground, exterior walls, corner markers,
// grid lines, corridor subdivision guides, a placeholder starting zone, and the
// central logo. It is then mutated by the per-scenario builders.
func GenerateBaseWorld(track *simconfig.Track, robot *simconfig.Robot) (root, world *Node) {
	root = New("sdf", "version", "1.7")
	world = root.Sub("world", "name", "wro_track_2026")

	addPhysics(world)
	addSunLight(world)
	addAmbientLight(world)
	addGround(world, track)
	addExteriorWalls(world, track)
	addCornerMarkers(world, track)
	addStartingZonePlaceholder(world, track)
	addCentralLogo(world, track)
	addGridLines(world, track)
	addCorridorSubdivisions(world, track)

	return root, world
}

func addPhysics(world *Node) {
	p := world.Sub("physics", "name", "default_physics", "default", "true", "type", "ode")
	p.SubT("max_step_size", ff(simconfig.PhysicsMaxStepSize))
	p.SubT("real_time_factor", ff(simconfig.PhysicsRealTimeFactor))
	p.SubT("real_time_update_rate", fi(simconfig.PhysicsUpdateRate))
}

func addSunLight(world *Node) {
	sun := world.Sub("light", "name", simconfig.ModelSunLight, "type", "directional")
	sun.SubT("pose", pose6(0, 0, simconfig.SunLightZ, 0, 0, 0))
	sun.SubT("diffuse", rgba(simconfig.SunDiffuseColor))
	sun.SubT("specular", rgba(simconfig.SunSpecularColor))
	sun.SubT("direction", vec3(
		simconfig.SunDefaultDirection[0],
		simconfig.SunDefaultDirection[1],
		simconfig.SunDefaultDirection[2],
	))
	sun.SubT("cast_shadows", "true")
}

func addAmbientLight(world *Node) {
	amb := world.Sub("light", "name", simconfig.ModelAmbientLight, "type", "point")
	amb.SubT("pose", pose6(0, 0, simconfig.AmbientLightZ, 0, 0, 0))
	amb.SubT("diffuse", rgba(simconfig.AmbientDiffuseColor))
	amb.SubT("specular", rgba(simconfig.AmbientSpecularColor))
	att := amb.Sub("attenuation")
	att.SubT("range", ff(simconfig.AmbientLightRange))
	att.SubT("constant", ff(simconfig.AmbientLightConstantAtten))
	att.SubT("linear", ff(simconfig.AmbientLightLinearAtten))
	att.SubT("quadratic", ff(simconfig.AmbientLightQuadraticAtten))
}

func addGround(world *Node, track *simconfig.Track) {
	model := world.Sub("model", "name", simconfig.ModelGround)
	model.SubT("static", "true")
	model.SubT("pose", pose6(track.TrackCenterCoord, track.TrackCenterCoord, 0, 0, 0, 0))
	link := model.Sub("link", "name", "link")

	planeSizeStr := fmt.Sprintf("%s %s", ff(track.TrackMatSize), ff(track.TrackMatSize))
	vis := link.Sub("visual", "name", "visual")
	groundPlane := vis.Sub("geometry").Sub("plane")
	groundPlane.SubT("normal", axisZ)
	groundPlane.SubT("size", planeSizeStr)
	mat := vis.Sub("material")
	gc := rgba(simconfig.GroundColor)
	mat.SubT("ambient", gc)
	mat.SubT("diffuse", gc)

	col := link.Sub("collision", "name", "collision")
	colPlane := col.Sub("geometry").Sub("plane")
	colPlane.SubT("normal", "0 0 1")
	colPlane.SubT("size", planeSizeStr)
	ode := col.Sub("surface").Sub("friction").Sub("ode")
	ode.SubT("mu", ff(simconfig.GroundFrictionMu))
	ode.SubT("mu2", ff(simconfig.GroundFrictionMu))
}

func addExteriorWalls(world *Node, track *simconfig.Track) {
	// Visual length = mat size + wall thickness; collision length adds extra on each side.
	visLen := track.TrackMatSize + track.WallThickness
	colLen := track.TrackMatSize + track.WallCollisionThickness
	h := track.WallHeight
	center := track.TrackCenterCoord
	outerEdge := track.TrackMaxCoord + track.WallThickness/2 // 3.05
	innerEdge := -track.WallThickness / 2                    // -0.05

	walls := []wallSpec{
		{simconfig.ModelExteriorWallNorth, center, outerEdge, visLen, h, colLen, track.WallCollisionThickness},
		{simconfig.ModelExteriorWallSouth, center, innerEdge, visLen, h, colLen, track.WallCollisionThickness},
		{simconfig.ModelExteriorWallEast, outerEdge, center, h, visLen, track.WallCollisionThickness, colLen},
		{simconfig.ModelExteriorWallWest, innerEdge, center, h, visLen, track.WallCollisionThickness, colLen},
	}

	for _, w := range walls {
		model := world.Sub("model", "name", w.name)
		model.SubT("static", "true")
		model.SubT("pose", pose6(w.cx, w.cy, h/2, 0, 0, 0))
		link := model.Sub("link", "name", "link")

		vis := link.Sub("visual", "name", "visual")
		vis.Sub("geometry").Sub("box").SubT("size", vec3(w.visX, w.visY, h))
		wc := rgba(track.WallColor)
		wallMat := vis.Sub("material")
		wallMat.SubT("ambient", wc)
		wallMat.SubT("diffuse", wc)

		col := link.Sub("collision", "name", "collision")
		col.Sub("geometry").Sub("box").SubT("size", vec3(w.colX, w.colY, h))
		ode := col.Sub("surface").Sub("contact").Sub("ode")
		ode.SubT("kp", ff(simconfig.WallContactKp))
		ode.SubT("kd", ff(simconfig.WallContactKd))
		ode.SubT("max_vel", ff(simconfig.WallContactMaxVel))
		ode.SubT("min_depth", ff(simconfig.WallContactMinDepth))
	}
}

func addCornerMarkers(world *Node, track *simconfig.Track) {
	blue := rgba(simconfig.CornerMarkerBlueColor)
	orange := rgba(simconfig.CornerMarkerOrangeColor)
	markerSize := vec3(simconfig.CornerMarkerLength, simconfig.CornerMarkerWidth, simconfig.CornerMarkerHeight)

	// Corner diagonal markers: two per corner (one blue, one orange).
	// Positions and rotations are fixed by WRO field geometry (π/6 increments).
	for _, c := range simconfig.CornerMarkers(track) {
		color := orange
		if c.Blue {
			color = blue
		}
		model := world.Sub("model", "name", c.Name)
		model.SubT("static", "true")
		model.SubT("pose", pose6(c.CX, c.CY, simconfig.ZGridLines, 0, 0, c.YawRad))
		vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
		vis.Sub("geometry").Sub("box").SubT("size", markerSize)
		mat := vis.Sub("material")
		mat.SubT("ambient", color)
		mat.SubT("diffuse", color)
	}
}

func addStartingZonePlaceholder(world *Node, track *simconfig.Track) {
	model := world.Sub("model", "name", simconfig.ModelStartingZonePlaceholder)
	model.SubT("static", "true")
	model.SubT("pose", pose6(
		track.TrackCenterCoord, track.TrackCornerMin/2, simconfig.ZStartingZoneBase, 0, 0, 0,
	))
	sizeStr := vec3(track.StartingZoneDefaultLength, track.StartingZoneWidth, track.StartingZoneThickness)
	vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", sizeStr)
	mat := vis.Sub("material")
	pc := rgba(simconfig.StartingZonePlaceholderColor)
	mat.SubT("ambient", pc)
	mat.SubT("diffuse", pc)
}

func addCentralLogo(world *Node, track *simconfig.Track) {
	model := world.Sub("model", "name", simconfig.ModelCentralLogo)
	model.SubT("static", "true")
	model.SubT("pose", pose6(track.TrackCenterCoord, track.TrackCenterCoord, simconfig.ZGridLines, 0, 0, 0))
	sizeStr := vec3(simconfig.CentralLogoSize, simconfig.CentralLogoSize, simconfig.GridLineHeight)
	vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", sizeStr)
	mat := vis.Sub("material")
	lc := rgba(simconfig.CentralLogoColor)
	mat.SubT("ambient", lc)
	mat.SubT("diffuse", lc)
}

func addGridLines(world *Node, track *simconfig.Track) {
	color := rgba(simconfig.GridLineColor)
	trackLen := ff(track.TrackSize)
	center := track.TrackCenterCoord
	thick := ff(simconfig.GridLineThickness)
	height := ff(simconfig.GridLineHeight)
	z := ff(simconfig.ZGridLines)

	// Vertical lines at x = TrackCornerMin (1.0) and TrackCornerMax (2.0)
	for _, line := range []gridLine{
		{simconfig.ModelGridLinePrefix + "v1", track.TrackCornerMin},
		{simconfig.ModelGridLinePrefix + "v2", track.TrackCornerMax},
	} {
		sizeStr := fmt.Sprintf("%s %s %s", thick, trackLen, height)
		addThinBoxModel(world, line.name, line.coord, center, z, sizeStr, color)
	}
	// Horizontal lines at y = TrackCornerMin (1.0) and TrackCornerMax (2.0)
	for _, line := range []gridLine{
		{simconfig.ModelGridLinePrefix + "h1", track.TrackCornerMin},
		{simconfig.ModelGridLinePrefix + "h2", track.TrackCornerMax},
	} {
		sizeStr := fmt.Sprintf("%s %s %s", trackLen, thick, height)
		addThinBoxModel(world, line.name, center, line.coord, z, sizeStr, color)
	}
}

func addCorridorSubdivisions(world *Node, track *simconfig.Track) {
	color := rgba(simconfig.CorridorSubdivisionColor)
	thick := ff(simconfig.SubdivLineThickness)
	height := ff(simconfig.SubdivLineHeight)
	z := ff(simconfig.ZGridLines)
	span := ff(track.TrackCornerSize) // 1.0 m — width of the inner square
	center := track.TrackCenterCoord

	southMid := track.TrackCornerMin / 2                      // 0.5
	northMid := track.TrackCornerMax + track.TrackCornerMin/2 // 2.5
	divOuter := track.CorridorDivOuter                        // 0.4
	divInner := track.CorridorDivInner                        // 0.6
	farDivOuter := track.TrackMaxCoord - divOuter             // 2.6
	farDivInner := track.TrackMaxCoord - divInner             // 2.4

	// NS corridors: center line is thin along X, span along Y; width markers are reversed.
	nsCenter := fmt.Sprintf("%s %s %s", thick, span, height)
	nsWidth := fmt.Sprintf("%s %s %s", span, thick, height)
	// EW corridors: swap X and Y dimensions.
	ewCenter := nsWidth
	ewWidth := nsCenter

	specs := []subdivisionSpec{
		{simconfig.SectionSouth, southMid, divOuter, divInner},
		{simconfig.SectionNorth, northMid, farDivInner, farDivOuter},
		{simconfig.SectionEast, northMid, farDivInner, farDivOuter},
		{simconfig.SectionWest, southMid, divOuter, divInner},
	}
	for _, sp := range specs {
		prefix := simconfig.ModelCorridorSubdivPrefix + string(sp.section) + "_"
		isNS := sp.section == simconfig.SectionNorth || sp.section == simconfig.SectionSouth
		if isNS {
			addThinBoxModel(world, prefix+"center", center, sp.mid, z, nsCenter, color)
			addThinBoxModel(world, prefix+"width1", center, sp.div1, z, nsWidth, color)
			addThinBoxModel(world, prefix+"width2", center, sp.div2, z, nsWidth, color)
		} else {
			addThinBoxModel(world, prefix+"center", sp.mid, center, z, ewCenter, color)
			addThinBoxModel(world, prefix+"width1", sp.div1, center, z, ewWidth, color)
			addThinBoxModel(world, prefix+"width2", sp.div2, center, z, ewWidth, color)
		}
	}
}

func addThinBoxModel(world *Node, name string, cx, cy float64, zStr, sizeStr, colorStr string) {
	model := world.Sub("model", "name", name)
	model.SubT("static", "true")
	model.SubT("pose", fmt.Sprintf("%s %s %s 0 0 0", ff(cx), ff(cy), zStr))
	vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", sizeStr)
	mat := vis.Sub("material")
	mat.SubT("ambient", colorStr)
	mat.SubT("diffuse", colorStr)
}
