package sdf

import (
	"fmt"
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/simconfig"
)

// GenerateBaseWorld returns the SDF root and world nodes for the WRO 2026 track.
// The world includes physics, lights, ground, exterior walls, corner markers,
// grid lines, corridor subdivision guides, a placeholder starting zone, and the
// central logo. It is then mutated by the per-scenario builders.
func GenerateBaseWorld() (root, world *Node) {
	root = New("sdf", "version", "1.7")
	world = root.Sub("world", "name", "wro_track_2026")

	addPhysics(world)
	addSunLight(world)
	addAmbientLight(world)
	addGround(world)
	addExteriorWalls(world)
	addCornerMarkers(world)
	addStartingZonePlaceholder(world)
	addCentralLogo(world)
	addGridLines(world)
	addCorridorSubdivisions(world)

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

func addGround(world *Node) {
	model := world.Sub("model", "name", simconfig.ModelGround)
	model.SubT("static", "true")
	model.SubT("pose", pose6(simconfig.TrackCenterCoord, simconfig.TrackCenterCoord, 0, 0, 0, 0))
	link := model.Sub("link", "name", "link")

	planeSizeStr := fmt.Sprintf("%s %s", ff(simconfig.TrackMatSize), ff(simconfig.TrackMatSize))
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

func addExteriorWalls(world *Node) {
	// Visual length = mat size + wall thickness; collision length adds extra on each side.
	visLen := simconfig.TrackMatSize + simconfig.WallThickness
	colLen := simconfig.TrackMatSize + simconfig.WallCollisionThickness
	h := simconfig.WallHeight
	center := simconfig.TrackCenterCoord
	outerEdge := simconfig.TrackMaxCoord + simconfig.WallThickness/2 // 3.05
	innerEdge := -simconfig.WallThickness / 2                        // -0.05

	walls := []struct {
		name       string
		cx, cy     float64
		visX, visY float64
		colX, colY float64
	}{
		{simconfig.ModelExteriorWallNorth, center, outerEdge, visLen, h, colLen, simconfig.WallCollisionThickness},
		{simconfig.ModelExteriorWallSouth, center, innerEdge, visLen, h, colLen, simconfig.WallCollisionThickness},
		{simconfig.ModelExteriorWallEast, outerEdge, center, h, visLen, simconfig.WallCollisionThickness, colLen},
		{simconfig.ModelExteriorWallWest, innerEdge, center, h, visLen, simconfig.WallCollisionThickness, colLen},
	}

	for _, w := range walls {
		model := world.Sub("model", "name", w.name)
		model.SubT("static", "true")
		model.SubT("pose", pose6(w.cx, w.cy, h/2, 0, 0, 0))
		link := model.Sub("link", "name", "link")

		vis := link.Sub("visual", "name", "visual")
		vis.Sub("geometry").Sub("box").SubT("size", vec3(w.visX, w.visY, h))
		wc := rgba(simconfig.WallColor)
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

func addCornerMarkers(world *Node) {
	blue := rgba(simconfig.CornerMarkerBlueColor)
	orange := rgba(simconfig.CornerMarkerOrangeColor)
	markerSize := vec3(simconfig.CornerMarkerLength, simconfig.CornerMarkerWidth, simconfig.CornerMarkerHeight)

	// Corner diagonal markers: two per corner (one blue, one orange).
	// Positions and rotations are fixed by WRO field geometry (π/6 increments).
	// color and name first: minimizes GC scan region (strings before float64s).
	type cornerEntry struct {
		color string
		name  string
		cx    float64
		cy    float64
		yaw   float64
	}
	pi6 := math.Pi / 6 // 30°
	corners := []cornerEntry{
		{cx: 2.5, cy: 2.29, yaw: pi6, color: blue, name: simconfig.ModelCornerNEBlue},
		{cx: 2.29, cy: 2.5, yaw: pi6 * 2, color: orange, name: simconfig.ModelCornerNEOrange},
		{cx: 2.5, cy: 0.71, yaw: -pi6, color: orange, name: simconfig.ModelCornerSEOrange},
		{cx: 2.29, cy: 0.5, yaw: -pi6 * 2, color: blue, name: simconfig.ModelCornerSEBlue},
		{cx: 0.5, cy: 0.71, yaw: -pi6 * 5, color: blue, name: simconfig.ModelCornerSWBlue},
		{cx: 0.71, cy: 0.5, yaw: -pi6 * 4, color: orange, name: simconfig.ModelCornerSWOrange},
		{cx: 0.5, cy: 2.29, yaw: pi6 * 5, color: orange, name: simconfig.ModelCornerNWOrange},
		{cx: 0.71, cy: 2.5, yaw: pi6 * 4, color: blue, name: simconfig.ModelCornerNWBlue},
	}
	for _, c := range corners {
		model := world.Sub("model", "name", c.name)
		model.SubT("static", "true")
		model.SubT("pose", pose6(c.cx, c.cy, simconfig.ZGridLines, 0, 0, c.yaw))
		vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
		vis.Sub("geometry").Sub("box").SubT("size", markerSize)
		mat := vis.Sub("material")
		mat.SubT("ambient", c.color)
		mat.SubT("diffuse", c.color)
	}
}

func addStartingZonePlaceholder(world *Node) {
	model := world.Sub("model", "name", simconfig.ModelStartingZonePlaceholder)
	model.SubT("static", "true")
	model.SubT("pose", pose6(
		simconfig.TrackCenterCoord, 0.5, simconfig.ZStartingZoneBase, 0, 0, 0,
	))
	sizeStr := vec3(simconfig.StartingZoneDefaultLength, simconfig.StartingZoneWidth, simconfig.StartingZoneThickness)
	vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", sizeStr)
	mat := vis.Sub("material")
	pc := rgba(simconfig.StartingZonePlaceholderColor)
	mat.SubT("ambient", pc)
	mat.SubT("diffuse", pc)
}

func addCentralLogo(world *Node) {
	model := world.Sub("model", "name", simconfig.ModelCentralLogo)
	model.SubT("static", "true")
	model.SubT("pose", pose6(simconfig.TrackCenterCoord, simconfig.TrackCenterCoord, simconfig.ZGridLines, 0, 0, 0))
	sizeStr := vec3(simconfig.CentralLogoSize, simconfig.CentralLogoSize, simconfig.GridLineHeight)
	vis := model.Sub("link", "name", "link").Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", sizeStr)
	mat := vis.Sub("material")
	lc := rgba(simconfig.CentralLogoColor)
	mat.SubT("ambient", lc)
	mat.SubT("diffuse", lc)
}

func addGridLines(world *Node) {
	color := rgba(simconfig.GridLineColor)
	trackLen := ff(simconfig.TrackSize)
	center := simconfig.TrackCenterCoord
	thick := ff(simconfig.GridLineThickness)
	height := ff(simconfig.GridLineHeight)
	z := ff(simconfig.ZGridLines)

	// Vertical lines at x = TrackCornerMin (1.0) and TrackCornerMax (2.0)
	for _, item := range []struct {
		name string
		cx   float64
	}{
		{simconfig.ModelGridLinePrefix + "v1", simconfig.TrackCornerMin},
		{simconfig.ModelGridLinePrefix + "v2", simconfig.TrackCornerMax},
	} {
		sizeStr := fmt.Sprintf("%s %s %s", thick, trackLen, height)
		addThinBoxModel(world, item.name, item.cx, center, z, sizeStr, color)
	}
	// Horizontal lines at y = TrackCornerMin (1.0) and TrackCornerMax (2.0)
	for _, item := range []struct {
		name string
		cy   float64
	}{
		{simconfig.ModelGridLinePrefix + "h1", simconfig.TrackCornerMin},
		{simconfig.ModelGridLinePrefix + "h2", simconfig.TrackCornerMax},
	} {
		sizeStr := fmt.Sprintf("%s %s %s", trackLen, thick, height)
		addThinBoxModel(world, item.name, center, item.cy, z, sizeStr, color)
	}
}

func addCorridorSubdivisions(world *Node) {
	color := rgba(simconfig.CorridorSubdivisionColor)
	thick := ff(simconfig.SubdivLineThickness)
	height := ff(simconfig.SubdivLineHeight)
	z := ff(simconfig.ZGridLines)
	span := ff(simconfig.TrackCornerSize) // 1.0 m — width of the inner square
	center := simconfig.TrackCenterCoord

	southMid := simconfig.TrackCornerMin / 2                          // 0.5
	northMid := simconfig.TrackCornerMax + simconfig.TrackCornerMin/2 // 2.5
	divOuter := simconfig.CorridorDivOuter                            // 0.4
	divInner := simconfig.CorridorDivInner                            // 0.6
	farDivOuter := simconfig.TrackMaxCoord - divOuter                 // 2.6
	farDivInner := simconfig.TrackMaxCoord - divInner                 // 2.4

	// NS corridors: center line is thin along X, span along Y; width markers are reversed.
	nsCenter := fmt.Sprintf("%s %s %s", thick, span, height)
	nsWidth := fmt.Sprintf("%s %s %s", span, thick, height)
	// EW corridors: swap X and Y dimensions.
	ewCenter := nsWidth
	ewWidth := nsCenter

	type subdivSpec struct {
		section         simconfig.Section
		mid, div1, div2 float64
	}
	specs := []subdivSpec{
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
