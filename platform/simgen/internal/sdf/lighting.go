package sdf

import (
	"fmt"

	"voldemorbot/simgen/internal/simconfig"
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

// updateSunLight modifies the sun light element with new direction and shadow settings.
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

// updateAmbientLight modifies the ambient light element with new intensity.
func updateAmbientLight(ambient *Node, cfg simconfig.LightingConfig) {
	amb := clampF(cfg.AmbientIntensity, 0, 1)
	av := ff(amb)
	ambStr := fmt.Sprintf("%s %s %s 1", av, av, av)
	if d := ambient.Find("diffuse"); d != nil {
		d.Text = ambStr
	}
}

func clampF(v, lo, hi float64) float64 {
	if v < lo {
		return lo
	}
	if v > hi {
		return hi
	}
	return v
}
