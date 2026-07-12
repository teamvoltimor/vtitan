package generate

import (
	"math"
	"math/rand"

	"voldemorbot/gazebo/generator/internal/simconfig"
)

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

// RandomizeStartingConditions picks a random corridor, direction, and spawn position.
func (r *Randomizer) RandomizeStartingConditions(
	widths map[simconfig.Section]simconfig.CorridorWidth,
) simconfig.StartingConditions {
	allSections := simconfig.AllSections[:]
	section := allSections[r.rng.Intn(len(allSections))]
	direction := simconfig.AllDirections[r.rng.Intn(len(simconfig.AllDirections))]

	corridorWidth := widths[section].Width
	position := r.pickStartPosition(section, corridorWidth)
	yaw := computeStartingYaw(section, direction)

	return simconfig.StartingConditions{
		Direction:   direction,
		Section:     section,
		SectionName: section.Capitalized(),
		Position:    position,
		Yaw:         yaw,
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
	spacing := simconfig.ParkingSpacingFactor * simconfig.RobotWidth
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
func (r *Randomizer) GenerateStartingZone(
	section simconfig.Section,
	corridorWidth float64,
	parking *simconfig.ParkingConfig,
) simconfig.StartingZone {
	defaultLength := simconfig.StartingZoneDefaultLength

	if parking != nil {
		return zoneFromParking(section, parking, defaultLength)
	}

	// Open challenge: random width and length offset. The width offset is a
	// fraction of this corridor's own width (not a fixed absolute meter
	// value) so the spawn always keeps chassis clearance from both the inner
	// block and the outer wall, regardless of which width the corridor
	// randomizer assigned this section.
	fractions := simconfig.StartingZoneWidthFractions[:]
	widthOffset := fractions[r.rng.Intn(len(fractions))] * corridorWidth
	lengthOffsets := []float64{
		simconfig.GridLengthSectionLeft,
		simconfig.GridLengthSectionRight,
	}
	lengthOffset := lengthOffsets[r.rng.Intn(len(lengthOffsets))]

	zoneX, zoneY := computeZoneCoords(section, lengthOffset, widthOffset, simconfig.TrackMaxCoord)
	return simconfig.StartingZone{Length: defaultLength, X: zoneX, Y: zoneY}
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

// pickStartPosition selects a random spawn position within the given corridor.
func (r *Randomizer) pickStartPosition(section simconfig.Section, corridorWidth float64) simconfig.Vec2 {
	trackMax := simconfig.TrackMaxCoord
	center := simconfig.TrackCenterCoord
	offsets := simconfig.StartPositionOffsets[:]
	d := offsets[r.rng.Intn(len(offsets))]

	switch section {
	case simconfig.SectionNorth:
		return simconfig.Vec2{center + d, trackMax - corridorWidth/2}
	case simconfig.SectionSouth:
		return simconfig.Vec2{center + d, corridorWidth / 2}
	case simconfig.SectionEast:
		return simconfig.Vec2{trackMax - corridorWidth/2, center + d}
	default: // West
		return simconfig.Vec2{corridorWidth / 2, center + d}
	}
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
	type key struct {
		s simconfig.Section
		d simconfig.Direction
	}
	yawMap := map[key]float64{
		{simconfig.SectionSouth, simconfig.DirectionClockwise}:        math.Pi,
		{simconfig.SectionSouth, simconfig.DirectionCounterClockwise}: 0.0,
		{simconfig.SectionNorth, simconfig.DirectionClockwise}:        0.0,
		{simconfig.SectionNorth, simconfig.DirectionCounterClockwise}: math.Pi,
		{simconfig.SectionEast, simconfig.DirectionClockwise}:         -halfPi,
		{simconfig.SectionEast, simconfig.DirectionCounterClockwise}:  halfPi,
		{simconfig.SectionWest, simconfig.DirectionClockwise}:         halfPi,
		{simconfig.SectionWest, simconfig.DirectionCounterClockwise}:  -halfPi,
	}
	return yawMap[key{section, direction}]
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
