package generate

import (
	"math"

	"voldemorbot/gazebo/generator/internal/simconfig"
)

type (
	// Strategy decides how each parameter of a scenario is chosen.
	// Implementations can be fully random, deterministic, or mixed.
	Strategy interface {
		CorridorWidths() map[simconfig.Section]simconfig.CorridorWidth
		Lighting() simconfig.LightingConfig
		StartingConditions(widths map[simconfig.Section]simconfig.CorridorWidth) simconfig.StartingConditions
	}

	// FullRandomization delegates every decision to the Randomizer.
	FullRandomization struct{ r *Randomizer }

	// DeterministicDefaults returns fixed, reproducible values — useful for testing.
	DeterministicDefaults struct{}

	// PartialRandomization randomizes only the parameters flagged as true.
	PartialRandomization struct {
		r            *Randomizer
		RandWidths   bool
		RandLighting bool
		RandStarting bool
		defaults     DeterministicDefaults
	}
)

// NewFullRandomization creates a strategy that fully randomizes all parameters.
func NewFullRandomization(r *Randomizer) *FullRandomization { return &FullRandomization{r: r} }

func (f *FullRandomization) CorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	return f.r.RandomizeCorridorWidths()
}
func (f *FullRandomization) Lighting() simconfig.LightingConfig { return f.r.RandomizeLighting() }

func (f *FullRandomization) StartingConditions(
	w map[simconfig.Section]simconfig.CorridorWidth,
) simconfig.StartingConditions {
	return f.r.RandomizeStartingConditions(w)
}

func (DeterministicDefaults) CorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	result := make(map[simconfig.Section]simconfig.CorridorWidth, 4)
	for _, s := range simconfig.AllSections {
		result[s] = simconfig.CorridorWidth{Type: simconfig.WidthTypeWide, Width: simconfig.CorridorWide}
	}
	return result
}

func (DeterministicDefaults) Lighting() simconfig.LightingConfig {
	return simconfig.LightingConfig{
		Intensity:        simconfig.DefaultLightingIntensity,
		AmbientIntensity: simconfig.DefaultAmbientIntensity,
		Direction:        simconfig.DefaultSunDirection,
		CastShadows:      true,
		Scenario:         simconfig.DefaultLightingScenario,
	}
}

func (DeterministicDefaults) StartingConditions(
	_ map[simconfig.Section]simconfig.CorridorWidth,
) simconfig.StartingConditions {
	return simconfig.StartingConditions{
		Direction:   simconfig.DirectionClockwise,
		Section:     simconfig.SectionSouth,
		SectionName: simconfig.SectionSouth.Capitalized(),
		Position:    simconfig.Vec2{simconfig.DefaultSpawnX, simconfig.DefaultSpawnY},
		Yaw:         math.Pi,
	}
}

// NewPartialRandomization creates a strategy that selectively randomizes parameters.
func NewPartialRandomization(r *Randomizer, randWidths, randLighting, randStarting bool) *PartialRandomization {
	return &PartialRandomization{
		r:            r,
		RandWidths:   randWidths,
		RandLighting: randLighting,
		RandStarting: randStarting,
	}
}

func (p *PartialRandomization) CorridorWidths() map[simconfig.Section]simconfig.CorridorWidth {
	if p.RandWidths {
		return p.r.RandomizeCorridorWidths()
	}
	return p.defaults.CorridorWidths()
}

func (p *PartialRandomization) Lighting() simconfig.LightingConfig {
	if p.RandLighting {
		return p.r.RandomizeLighting()
	}
	return p.defaults.Lighting()
}

func (p *PartialRandomization) StartingConditions(
	w map[simconfig.Section]simconfig.CorridorWidth,
) simconfig.StartingConditions {
	if p.RandStarting {
		return p.r.RandomizeStartingConditions(w)
	}
	return p.defaults.StartingConditions(w)
}
