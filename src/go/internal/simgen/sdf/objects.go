package sdf

import (
	"fmt"

	"github.com/teamvoltimor/vtitan/src/go/internal/simgen/simconfig"
)

// parkingBlockSpec is one parking-limitation block model: its name,
// placement and heading.
type parkingBlockSpec struct {
	name string
	pos  simconfig.Vec2
	yaw  float64
}

// AddTrafficSigns appends one box model per sign to the world element.
// positions and colors must be the same length.
func AddTrafficSigns(world *Node, signs []simconfig.Sign) {
	for i, s := range signs {
		var prefix string
		if s.Color.Name == simconfig.ColorNameRed {
			prefix = simconfig.ModelRedSignPrefix
		} else {
			prefix = simconfig.ModelGreenSignPrefix
		}
		model := New("model", "name", fmt.Sprintf("%s%d", prefix, i))
		model.SubT("static", "true")
		model.SubT("pose", pose6(s.Position[0], s.Position[1], simconfig.SignZPosition, 0, 0, 0))
		link := model.Sub("link", "name", "link")
		AddBoxVisual(link, simconfig.SignWidth, simconfig.SignDepth, simconfig.SignHeight, s.Color.RGB, "visual")
		AddBoxCollision(link, simconfig.SignWidth, simconfig.SignDepth, simconfig.SignHeight, "collision")
		world.Add(model)
	}
}

// AddParkingLot appends the two parking limitation block models to the world element.
func AddParkingLot(world *Node, cfg simconfig.ParkingConfig) {
	parkColor := simconfig.ParkingColor
	for _, block := range []parkingBlockSpec{
		{simconfig.ModelParkingBlock1, cfg.Block1Pos, cfg.Block1Yaw},
		{simconfig.ModelParkingBlock2, cfg.Block2Pos, cfg.Block2Yaw},
	} {
		model := New("model", "name", block.name)
		model.SubT("static", "true")
		model.SubT("pose", pose6(block.pos[0], block.pos[1], simconfig.ParkingZPosition, 0, 0, block.yaw))
		link := model.Sub("link", "name", "link")
		AddBoxVisual(
			link,
			simconfig.ParkingLength,
			simconfig.ParkingWidth,
			simconfig.ParkingHeight,
			parkColor,
			"visual",
		)
		AddBoxCollision(link, simconfig.ParkingLength, simconfig.ParkingWidth, simconfig.ParkingHeight, "collision")
		world.Add(model)
	}
}
