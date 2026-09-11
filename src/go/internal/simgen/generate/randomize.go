package generate

import (
	"math"
	"math/rand"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// startKey is the (section, direction) pair a starting yaw is looked up by.
type startKey struct {
	s simconfig.Section
	d simconfig.Direction
}

// Randomizer generates all stochastic parameters for a WRO 2026 scenario.
// A single seeded *rand.Rand drives all randomness so generation is reproducible.
// Note: seed-for-seed output does not match the Python generator (Python uses
// separate random/numpy seeds with different PRNG algorithms).
type Randomizer struct {
	rng *rand.Rand
}

func NewRandomizer(rng *rand.Rand) *Randomizer {
	return &Randomizer{rng: rng}
}

// RandomizeLighting picks one of the six WRO lighting presets and samples
// intensity and direction within its spec ranges.
func (r *Randomizer) RandomizeLighting() simconfig.LightingConfig {
	all := simconfig.AllLightingScenarios
	scenario := all[r.rng.Intn(len(all))]
	spec := simconfig.LightingSpecs[scenario]
	return simconfig.LightingConfig{
		Intensity:        r.uniform(spec.IntensityMin, spec.IntensityMax),
		AmbientIntensity: r.uniform(spec.AmbientMin, spec.AmbientMax),
		Direction: [3]float64{
			r.uniform(spec.DirXMin, spec.DirXMax),
			r.uniform(spec.DirYMin, spec.DirYMax),
			spec.DirZ,
		},
		CastShadows: spec.CastShadows,
		Scenario:    string(scenario),
	}
}

// RandomizeCorridorWidths assigns each corridor randomly as narrow or wide
// (Open challenge). Obstacles widths are always fixed — use fixedCorridorWidths().
func (r *Randomizer) RandomizeCorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	result := make(map[simconfig.Section]simconfig.CorridorWidth, 4)
	types := []string{simconfig.WidthTypeNarrow, simconfig.WidthTypeWide}
	widths := map[string]float64{
		simconfig.WidthTypeNarrow: simconfig.CorridorNarrow,
		simconfig.WidthTypeWide:   simconfig.CorridorWide,
	}
	for _, s := range simconfig.AllSections {
		t := types[r.rng.Intn(2)]
		result[s] = simconfig.CorridorWidth{Type: t, Width: widths[t]}
	}
	return result
}

// FixedCorridorWidths returns 1.0 m corridors for all sections (obstacles challenge).
func FixedCorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	result := make(map[simconfig.Section]simconfig.CorridorWidth, 4)
	for _, s := range simconfig.AllSections {
		result[s] = simconfig.CorridorWidth{Type: simconfig.WidthTypeFixed, Width: simconfig.CorridorObstacles}
	}
	return result
}

// RandomizeStartingConditions picks a random corridor, direction, and starting
// cell. The spawn pose is the chosen cell's centre — one of the mat's real
// starting positions — not the corridor centreline, which is not a legal start
// and was the only pose the generator ever emitted.
func (r *Randomizer) RandomizeStartingConditions(
	widths map[simconfig.Section]simconfig.CorridorWidth,
) simconfig.StartingConditions {
	allSections := simconfig.AllSections[:]
	section := allSections[r.rng.Intn(len(allSections))]
	direction := simconfig.AllDirections[r.rng.Intn(len(simconfig.AllDirections))]

	cells := StartCells(section, widths[section].Width)
	cell := r.rng.Intn(len(cells))
	yaw := computeStartingYaw(section, direction)

	return simconfig.StartingConditions{
		Direction:   direction,
		Section:     section,
		SectionName: section.Capitalized(),
		Position:    cells[cell].Spawn,
		Yaw:         yaw,
		StartCell:   cell,
	}
}

// GenerateParkingLotPositions places the two magenta parking blocks in the
// starting section's corner (obstacles challenge only).
func (r *Randomizer) GenerateParkingLotPositions(startSection simconfig.Section) simconfig.ParkingConfig {
	depthChoices := []float64{
		simconfig.SignGridDepthNear,
		simconfig.SignGridDepthMiddle,
		simconfig.SignGridDepthFar,
	}
	depth := depthChoices[r.rng.Intn(len(depthChoices))]
	// Along-travel gap between the two blocks — the actual usable bay length
	// the robot must pull into. Must scale with RobotLength (the dimension
	// that has to fit inside the bay), not RobotWidth: at RobotWidth (0.2m)
	// this came out to exactly RobotLength (0.3m), a zero-clearance bay the
	// robot could never actually enter.
	spacing := simconfig.ParkingSpacingFactor * simconfig.RobotLength
	depth2 := r.computeSecondBlockDepth(depth, spacing)

	b1, b2, yaw := parkingPositionsForSection(startSection, depth, depth2, simconfig.ParkingWallOffset)
	return simconfig.ParkingConfig{
		Block1Pos: b1,
		Block2Pos: b2,
		Block1Yaw: yaw,
		Block2Yaw: yaw,
		Depth:     depth,
	}
}

// GenerateStartingZone computes the starting zone rectangle and position.
//
// It consumes no randomness: the Open Challenge zone is the cell already drawn
// by RandomizeStartingConditions, looked up by index. Deriving both the zone
// and the spawn pose from that single draw is what keeps them consistent —
// they used to be randomized independently, and AddStartingZone then silently
// overwrote the spawn with the zone centre.
func GenerateStartingZone(
	section simconfig.Section,
	corridorWidth float64,
	startCell int,
	parking *simconfig.ParkingConfig,
) simconfig.StartingZone {
	defaultLength := simconfig.StartingZoneDefaultLength

	if parking != nil {
		zone := zoneFromParking(section, parking, defaultLength)
		zone.Width = simconfig.StartingZoneWidth
		return zone
	}

	cells := StartCells(section, corridorWidth)
	cell := cells[startCell%len(cells)]
	return simconfig.StartingZone{
		Length: defaultLength,
		Width:  cell.BandWidth,
		X:      cell.ZoneCentre[0],
		Y:      cell.ZoneCentre[1],
	}
}

// computeZoneCoords determines the (x, y) position for a starting zone given a section and offsets.
func computeZoneCoords(section simconfig.Section, lengthOffset, widthOffset, trackMax float64) (x, y float64) {
	isNS := section == simconfig.SectionNorth || section == simconfig.SectionSouth
	invertWidth := section == simconfig.SectionNorth || section == simconfig.SectionEast

	if isNS {
		x = lengthOffset
		y = widthOffset
		if invertWidth {
			y = trackMax - widthOffset
		}
	} else {
		y = lengthOffset
		x = widthOffset
		if invertWidth {
			x = trackMax - widthOffset
		}
	}
	return
}

// GenerateSignPositions places traffic signs using the 36-scenario system.
// excludeSection is skipped (the starting section in obstacles challenge).
// Returns one Sign per pillar with mean color values (no per-sign Gaussian noise).
func (r *Randomizer) GenerateSignPositions(
	_ map[simconfig.Section]simconfig.CorridorWidth,
	excludeSection *simconfig.Section,
) []simconfig.Sign {
	var signs []simconfig.Sign
	for _, section := range simconfig.AllSections {
		if excludeSection != nil && section == *excludeSection {
			continue
		}
		scenarioID := r.rng.Intn(simconfig.ScenarioIDMax-simconfig.ScenarioIDMin+1) + simconfig.ScenarioIDMin
		pillars, err := ApplyScenarioToSection(scenarioID, section)
		if err != nil {
			continue
		}
		for _, p := range pillars {
			var colorRGB simconfig.RGB
			var colorName string
			if p.Color == simconfig.ColorNameRed {
				colorRGB = simconfig.SignColorRed
				colorName = simconfig.ColorNameRed
			} else {
				colorRGB = simconfig.SignColorGreen
				colorName = simconfig.ColorNameGreen
			}
			signs = append(signs, simconfig.Sign{
				Position: simconfig.Vec2{p.X, p.Y},
				Color:    simconfig.SignColor{Name: colorName, RGB: colorRGB},
			})
		}
	}
	return signs
}

// Private helpers

// uniform samples uniformly from [lo, hi).
func (r *Randomizer) uniform(lo, hi float64) float64 {
	return lo + r.rng.Float64()*(hi-lo)
}

// computeSecondBlockDepth calculates the depth position of the second parking block given the first block's depth.
func (r *Randomizer) computeSecondBlockDepth(depth, spacing float64) float64 {
	near := simconfig.SignGridDepthNear
	far := simconfig.SignGridDepthFar
	if depth == near {
		return depth + spacing
	}
	if depth == far {
		return depth - spacing
	}
	if r.rng.Float64() < 0.5 {
		return depth + spacing
	}
	return depth - spacing
}

// computeStartingYaw calculates the robot's initial yaw angle based on its corridor and traversal direction.
func computeStartingYaw(section simconfig.Section, direction simconfig.Direction) float64 {
	halfPi := math.Pi / 2
	yawMap := map[startKey]float64{
		{simconfig.SectionSouth, simconfig.DirectionClockwise}:        math.Pi,
		{simconfig.SectionSouth, simconfig.DirectionCounterClockwise}: 0.0,
		{simconfig.SectionNorth, simconfig.DirectionClockwise}:        0.0,
		{simconfig.SectionNorth, simconfig.DirectionCounterClockwise}: math.Pi,
		{simconfig.SectionEast, simconfig.DirectionClockwise}:         -halfPi,
		{simconfig.SectionEast, simconfig.DirectionCounterClockwise}:  halfPi,
		{simconfig.SectionWest, simconfig.DirectionClockwise}:         halfPi,
		{simconfig.SectionWest, simconfig.DirectionCounterClockwise}:  -halfPi,
	}
	return yawMap[startKey{section, direction}]
}

// parkingPositionsForSection computes the world coordinates of the two parking blocks for a given section.
func parkingPositionsForSection(
	section simconfig.Section,
	depth, depth2, wallOffset float64,
) (pos1, pos2 simconfig.Vec2, yaw float64) {
	halfPi := math.Pi / 2
	trackMax := simconfig.TrackMaxCoord
	switch section {
	case simconfig.SectionSouth:
		return simconfig.Vec2{depth, wallOffset}, simconfig.Vec2{depth2, wallOffset}, halfPi
	case simconfig.SectionNorth:
		y := trackMax - wallOffset
		return simconfig.Vec2{depth, y}, simconfig.Vec2{depth2, y}, halfPi
	case simconfig.SectionEast:
		x := trackMax - wallOffset
		return simconfig.Vec2{x, depth}, simconfig.Vec2{x, depth2}, 0.0
	default: // West
		return simconfig.Vec2{wallOffset, depth}, simconfig.Vec2{wallOffset, depth2}, 0.0
	}
}

// zoneFromParking computes the starting zone position and size based on parking block positions.
//
// The along-travel coordinate (X for a north/south section, Y for east/west)
// comes from the parking blocks' own midpoint, so the robot starts near where
// it will eventually park. The cross-corridor coordinate deliberately does
// NOT reuse the parking blocks' own position: they sit ParkingWallOffset
// (0.10m — exactly RobotWidth/2) from the outer wall, which is correct for a
// thin parking marker but leaves zero chassis clearance for the robot itself.
// Use the corridor centerline instead, which — for the fixed-width Obstacles
// Challenge corridor — comfortably clears both the inner block and the outer
// wall.
func zoneFromParking(
	section simconfig.Section,
	parking *simconfig.ParkingConfig,
	defaultLength float64,
) simconfig.StartingZone {
	b1, b2 := parking.Block1Pos, parking.Block2Pos
	isNS := section == simconfig.SectionNorth || section == simconfig.SectionSouth
	invertWidth := section == simconfig.SectionNorth || section == simconfig.SectionEast

	corridorCenter := simconfig.CorridorObstacles / 2
	widthCoord := corridorCenter
	if invertWidth {
		widthCoord = simconfig.TrackMaxCoord - corridorCenter
	}

	var spacing, zoneX, zoneY float64
	if isNS {
		spacing = math.Abs(b2[0] - b1[0])
		zoneX = (b1[0] + b2[0]) / 2
		zoneY = widthCoord
	} else {
		spacing = math.Abs(b2[1] - b1[1])
		zoneY = (b1[1] + b2[1]) / 2
		zoneX = widthCoord
	}

	availableGap := spacing - simconfig.ParkingWidth
	length := defaultLength
	if availableGap < defaultLength {
		candidate := availableGap * simconfig.StartingZoneObstaclesFactor
		if candidate < length {
			length = candidate
		}
	}
	return simconfig.StartingZone{Length: length, X: zoneX, Y: zoneY}
}
