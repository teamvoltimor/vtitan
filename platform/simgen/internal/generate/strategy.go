package generate

import (
	"math"

	"voldemorbot/simgen/internal/simconfig"
)

// Strategy decides how each parameter of a scenario is chosen.
// Implementations can be fully random, deterministic, or mixed.
type Strategy interface {
	CorridorWidths() map[simconfig.Section]simconfig.CorridorWidth
	Lighting() simconfig.LightingConfig
	StartingConditions(widths map[simconfig.Section]simconfig.CorridorWidth) simconfig.StartingConditions
}

// FullRandomization delegates every decision to the Randomizer.
type FullRandomization struct{ r *Randomizer }

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

// DeterministicDefaults returns fixed, reproducible values — useful for testing.
type DeterministicDefaults struct{}

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

// PartialRandomization randomizes only the parameters flagged as true.
type PartialRandomization struct {
	r            *Randomizer
	RandWidths   bool
	RandLighting bool
	RandStarting bool
	defaults     DeterministicDefaults
}

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
