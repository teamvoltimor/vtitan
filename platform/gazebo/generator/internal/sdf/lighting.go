package sdf

import (
	"fmt"

	"voldemorbot/gazebo/generator/internal/simconfig"
)

// ApplyLighting updates the sun and ambient light elements in the world node
// from a resolved LightingConfig.
func ApplyLighting(world *Node, cfg simconfig.LightingConfig) {
	intensity := clampF(cfg.Intensity, 0, 1)
	iv := ff(intensity)
	diffuse := fmt.Sprintf("%s %s %s 1", iv, iv, iv)

	if sun := world.FindAttr("light", "name", simconfig.ModelSunLight); sun != nil {
		updateSunLight(sun, diffuse, cfg)
	}

	if ambient := world.FindAttr("light", "name", simconfig.ModelAmbientLight); ambient != nil {
		updateAmbientLight(ambient, cfg)
	}
}

// updateSunLight updates the sun light's diffuse color, direction, and shadow casting.
func updateSunLight(sun *Node, diffuse string, cfg simconfig.LightingConfig) {
	if d := sun.Find("diffuse"); d != nil {
		d.Text = diffuse
	}
	if dir := sun.Find("direction"); dir != nil {
		dir.Text = vec3(cfg.Direction[0], cfg.Direction[1], cfg.Direction[2])
	}
	if cs := sun.Find("cast_shadows"); cs != nil {
		if cfg.CastShadows {
			cs.Text = "true"
		} else {
			cs.Text = "false"
		}
	}
}

// updateAmbientLight updates the ambient light's diffuse color.
func updateAmbientLight(ambient *Node, cfg simconfig.LightingConfig) {
	amb := clampF(cfg.AmbientIntensity, 0, 1)
	av := ff(amb)
	ambStr := fmt.Sprintf("%s %s %s 1", av, av, av)
	if d := ambient.Find("diffuse"); d != nil {
		d.Text = ambStr
	}
}

// clampF clamps a value v to the range [lo, hi].
func clampF(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
