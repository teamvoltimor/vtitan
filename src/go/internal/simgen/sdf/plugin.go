package sdf

import "github.com/teamvoltimor/vtitan/platform/robot-go/internal/simgen/simconfig"

// AddSystemPlugins injects the Sensors and Physics system plugins into a world
// element, skipping any that are already present.
func AddSystemPlugins(world *Node) {
	if world.FindAttr("plugin", "name", simconfig.PluginSensorsName) == nil {
		sensors := New("plugin",
			"filename", simconfig.PluginSensorsFilename,
			"name", simconfig.PluginSensorsName,
		)
		sensors.SubT("render_engine", simconfig.RenderEngineOgre2)
		world.Insert(0, sensors)
	}
	if world.FindAttr("plugin", "name", simconfig.PluginPhysicsName) == nil {
		physics := New("plugin",
			"filename", simconfig.PluginPhysicsFilename,
			"name", simconfig.PluginPhysicsName,
		)
		world.Insert(1, physics)
	}
}
