package sdf

import (
	"fmt"

	"vtitan/gazebo/generator/internal/simconfig"
)

// AddInteriorWalls computes the four interior wall positions from corridor widths
// and appends them to the world element.
func AddInteriorWalls(world *Node, corridorWidths map[simconfig.Section]simconfig.CorridorWidth) {
	trackMax := simconfig.TrackMaxCoord

	northY := trackMax - corridorWidths[simconfig.SectionNorth].Width
	southY := corridorWidths[simconfig.SectionSouth].Width
	eastX := trackMax - corridorWidths[simconfig.SectionEast].Width
	westX := corridorWidths[simconfig.SectionWest].Width

	// Each wall: (name, cx, cy, visualX, visualY, isHorizontal)
	walls := []struct {
		name       string
		cx, cy     float64
		vx, vy     float64
		horizontal bool
	}{
		{
			simconfig.ModelInteriorWallNorth,
			(eastX + westX) / 2,
			northY - simconfig.WallInteriorOffset,
			eastX - westX, simconfig.WallThickness,
			true,
		},
		{
			simconfig.ModelInteriorWallSouth,
			(eastX + westX) / 2,
			southY + simconfig.WallInteriorOffset,
			eastX - westX, simconfig.WallThickness,
			true,
		},
		{
			simconfig.ModelInteriorWallEast,
			eastX - simconfig.WallInteriorOffset,
			(northY + southY) / 2,
			simconfig.WallThickness, northY - southY,
			false,
		},
		{
			simconfig.ModelInteriorWallWest,
			westX + simconfig.WallInteriorOffset,
			(northY + southY) / 2,
			simconfig.WallThickness, northY - southY,
			false,
		},
	}

	for _, w := range walls {
		var colX, colY float64
		if w.horizontal {
			colX, colY = w.vx, simconfig.WallCollisionThickness
		} else {
			colX, colY = simconfig.WallCollisionThickness, w.vy
		}
		world.Add(BuildWallModel(w.name, w.cx, w.cy, w.vx, w.vy, colX, colY))
	}
}

// AddStartingZone removes any existing starting zone placeholder from the base
// template, then adds the dynamically positioned zone rectangle and direction
// indicator.
//
// It does not touch sc.Position. It used to overwrite it with the zone centre,
// which made the generator's own spawn selection dead code and, once the spawn
// stopped being the centre of its cell, would have silently discarded the
// chosen StartingZoneSpawnOffsets. CreateScenario owns the spawn pose.
func AddStartingZone(
	world *Node,
	sc *simconfig.StartingConditions,
	corridorWidths map[simconfig.Section]simconfig.CorridorWidth,
) {
	zone := sc.Zone

	// Remove the static placeholder from the base template
	if existing := world.FindAttr("model", "name", simconfig.ModelStartingZonePlaceholder); existing != nil {
		world.RemoveChild(existing)
	}

	isNS := sc.Section == simconfig.SectionNorth || sc.Section == simconfig.SectionSouth
	// Cross-corridor size comes from the zone, not a fixed constant: an Open
	// Challenge zone is one cell of the starting square and is as wide as the
	// band that cell sits in (0.40 or 0.20 m), so painting every zone 0.20 m
	// wide would misdraw four of the six cells.
	zoneWidth := zone.Width
	if zoneWidth <= 0 {
		zoneWidth = simconfig.StartingZoneWidth
	}
	var zoneSizeStr string
	if isNS {
		zoneSizeStr = vec3(zone.Length, zoneWidth, simconfig.StartingZoneThickness)
	} else {
		zoneSizeStr = vec3(zoneWidth, zone.Length, simconfig.StartingZoneThickness)
	}

	var indicatorRGB simconfig.RGB
	if sc.Direction == simconfig.DirectionClockwise {
		indicatorRGB = simconfig.StartingZoneClockwiseColor
	} else {
		indicatorRGB = simconfig.StartingZoneCounterClockwiseColor
	}

	modelName := fmt.Sprintf("%s%s", simconfig.ModelStartingZonePrefix, sc.Section)
	model := New("model", "name", modelName)
	model.SubT("static", "true")
	model.SubT("pose", pose6(zone.X, zone.Y, simconfig.ZStartingZoneBase, 0, 0, 0))

	link := model.Sub("link", "name", "link")

	// Grey base rectangle
	visBase := link.Sub("visual", "name", "visual_base")
	visBase.Sub("geometry").Sub("box").SubT("size", zoneSizeStr)
	matBase := visBase.Sub("material")
	bc := rgba(simconfig.StartingZoneColor)
	matBase.SubT("ambient", bc)
	matBase.SubT("diffuse", bc)

	// Colored direction indicator circle
	visInd := link.Sub("visual", "name", "visual_direction_indicator")
	visInd.SubT("pose", pose6(0, 0, simconfig.ZDirectionIndicator, 0, 0, 0))
	cyl := visInd.Sub("geometry").Sub("cylinder")
	cyl.SubT("radius", ff(simconfig.StartingZoneIndicatorRadius))
	cyl.SubT("length", ff(simconfig.StartingZoneThickness))
	matInd := visInd.Sub("material")
	ic := rgba(indicatorRGB)
	matInd.SubT("ambient", ic)
	matInd.SubT("diffuse", ic)
	matInd.SubT("emissive", ic)

	world.Add(model)
}
