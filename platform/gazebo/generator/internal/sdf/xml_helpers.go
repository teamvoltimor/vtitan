package sdf

import (
	"fmt"
	"strconv"

	"voldemorbot/gazebo/generator/internal/simconfig"
)

// Axis direction constants for joint and geometry normal definitions.
const (
	axisX = "1 0 0"
	axisY = "0 1 0"
	axisZ = "0 0 1"
)

// Joint limit sentinels for continuous revolute joints (±1e16 = no limit in Gazebo SDF).
const (
	jointLimitUnbounded    = "1e16"
	jointLimitUnboundedNeg = "-1e16"
)

// Zero-value string constants for SDF elements.
const (
	floatZero = "0.0"
	intZero   = "0"
)

// ff formats a float64 as a decimal string without trailing zeros.
func ff(f float64) string {
	return strconv.FormatFloat(f, 'f', -1, 64)
}

// fi formats an integer as a decimal string.
func fi(i int) string {
	return strconv.Itoa(i)
}

// pose6 formats a 6-DOF pose string "x y z roll pitch yaw".
func pose6(x, y, z, roll, pitch, yaw float64) string {
	return fmt.Sprintf("%s %s %s %s %s %s", ff(x), ff(y), ff(z), ff(roll), ff(pitch), ff(yaw))
}

// poseXY formats a pose at (x, y, 0) with zero rotation.
func poseXY(x, y float64) string {
	return fmt.Sprintf("%s %s 0 0 0 0", ff(x), ff(y))
}

// vec3 formats three floats as "x y z".
func vec3(x, y, z float64) string {
	return fmt.Sprintf("%s %s %s", ff(x), ff(y), ff(z))
}

// rgba formats an RGB triple plus alpha 1 as "r g b 1".
func rgba(c simconfig.RGB) string {
	return fmt.Sprintf("%s %s %s 1", ff(c[0]), ff(c[1]), ff(c[2]))
}

// AddBoxVisual appends a box <visual> element to a link node.
func AddBoxVisual(link *Node, width, depth, height float64, color simconfig.RGB, visualName string) {
	if visualName == "" {
		visualName = "visual"
	}
	vis := link.Sub("visual", "name", visualName)
	vis.Sub("geometry").Sub("box").SubT("size", vec3(width, depth, height))
	mat := vis.Sub("material")
	c := rgba(color)
	mat.SubT("ambient", c)
	mat.SubT("diffuse", c)
}

// AddBoxCollision appends a box <collision> element to a link node.
func AddBoxCollision(link *Node, width, depth, height float64, collisionName string) {
	if collisionName == "" {
		collisionName = "collision"
	}
	col := link.Sub("collision", "name", collisionName)
	col.Sub("geometry").Sub("box").SubT("size", vec3(width, depth, height))
}

// BuildWallModel constructs a static wall <model> element from center coords and dimensions.
func BuildWallModel(name string, cx, cy, visualX, visualY, collisionX, collisionY float64) *Node {
	model := New("model", "name", name)
	model.SubT("static", "true")
	model.SubT("pose", pose6(cx, cy, 0, 0, 0, 0))
	link := model.Sub("link", "name", "link")
	AddBoxVisual(link, visualX, visualY, simconfig.WallHeight, simconfig.WallColor, "visual")
	AddBoxCollision(link, collisionX, collisionY, simconfig.WallHeight, "collision")
	return model
}
