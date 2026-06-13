package sdf

import (
	"fmt"
	"math"

	"voldemorbot/simgen/internal/simconfig"
)

// wheelRollPose is the 6-DOF pose that rotates a cylinder 90° around X so its
// axis aligns with Y (the wheel roll axis in Gazebo).
var wheelRollPose = pose6(0, 0, 0, math.Pi/2, 0, 0)

// AddRobotModel appends the full WRO robot model to the world element and
// also inserts the static overhead debug camera as a sibling world model.
func AddRobotModel(world *Node, sc simconfig.StartingConditions) {
	robot := New("model", "name", simconfig.ModelRobotName)
	robot.SubT("pose", pose6(sc.Position[0], sc.Position[1], simconfig.RobotWheelRadius, 0, 0, sc.Yaw))

	buildChassis(robot)
	buildAckermannPlugin(robot)
	buildRearWheels(robot)
	buildFrontSteering(robot)
	buildCameraLink(robot)
	buildLidarLink(robot)
	buildImuLink(robot)

	world.Add(robot)
	buildDebugOverheadCamera(world)
}

func buildChassis(robot *Node) {
	halfH := simconfig.RobotHeight / 2
	cm := simconfig.RobotChassisMass
	// Box inertia tensor (uniform density).
	ixx := (1.0 / 12) * cm * (sq(simconfig.RobotWidth) + sq(simconfig.RobotHeight))
	iyy := (1.0 / 12) * cm * (sq(simconfig.RobotLength) + sq(simconfig.RobotHeight))
	izz := (1.0 / 12) * cm * (sq(simconfig.RobotLength) + sq(simconfig.RobotWidth))

	base := robot.Sub("link", "name", simconfig.RobotBaseFrameID)

	// Main chassis box — blue
	vis := base.Sub("visual", "name", "visual")
	vis.SubT("pose", pose6(0, 0, halfH, 0, 0, 0))
	vis.Sub("geometry").
		Sub("box").
		SubT("size", vec3(simconfig.RobotLength, simconfig.RobotWidth, simconfig.RobotHeight))
	setMaterial(vis, simconfig.RobotChassisColor)

	// Red front indicator (sits above LIDAR scan plane so it is always visible).
	frontX := simconfig.RobotLength/2 - simconfig.RobotFrontIndicatorOffsetX
	frontZ := simconfig.RobotHeight + simconfig.RobotFrontIndicatorOffsetZ
	ind := base.Sub("visual", "name", "front_indicator")
	ind.SubT("pose", pose6(frontX, 0, frontZ, 0, 0, 0))
	sz := simconfig.RobotFrontIndicatorSize
	ind.Sub("geometry").Sub("box").SubT("size", vec3(sz[0], sz[1], sz[2]))
	setMaterial(ind, simconfig.RobotFrontIndicatorColor)

	col := base.Sub("collision", "name", "collision")
	col.SubT("pose", pose6(0, 0, halfH, 0, 0, 0))
	col.Sub("geometry").
		Sub("box").
		SubT("size", vec3(simconfig.RobotLength, simconfig.RobotWidth, simconfig.RobotHeight))

	inertial := base.Sub("inertial")
	inertial.SubT("mass", ff(cm))
	ix := inertial.Sub("inertia")
	ix.SubT("ixx", fmt.Sprintf("%.6f", ixx))
	ix.SubT("iyy", fmt.Sprintf("%.6f", iyy))
	ix.SubT("izz", fmt.Sprintf("%.6f", izz))
}

func buildAckermannPlugin(robot *Node) {
	plugin := robot.Sub("plugin",
		"filename", simconfig.PluginAckermannFilename,
		"name", simconfig.PluginAckermannName,
	)
	params := [][2]string{
		{"left_joint", "rear_left_wheel_joint"},
		{"right_joint", "rear_right_wheel_joint"},
		{"left_steering_joint", "front_left_steering_joint"},
		{"right_steering_joint", "front_right_steering_joint"},
		{"wheel_separation", ff(simconfig.RobotTrackWidth)},
		{"kingpin_width", ff(simconfig.RobotTrackWidth)},
		{"wheel_base", ff(simconfig.RobotWheelbase)},
		{"wheel_radius", ff(simconfig.RobotWheelRadius)},
		{"min_steering_angle", "-" + ff(simconfig.RobotMaxSteering)},
		{"max_steering_angle", ff(simconfig.RobotMaxSteering)},
		{"topic", simconfig.RobotCmdVelTopic},
		{"odom_topic", simconfig.RobotOdomTopic},
		{"odom_publish_frequency", simconfig.RobotOdomFrequency},
		{"frame_id", simconfig.RobotOdomFrameID},
		{"child_frame_id", simconfig.RobotBaseFrameID},
	}
	for _, p := range params {
		plugin.SubT(p[0], p[1])
	}
}

func buildWheelLink(parent *Node, name, poseText string) {
	r := simconfig.RobotWheelRadius
	w := simconfig.RobotWheelWidth
	wm := simconfig.RobotWheelMass

	// Cylinder inertia (solid, roll axis = Y after wheelRollPose rotation).
	wheelIxx := (1.0 / 12) * wm * (3*sq(r) + sq(w))
	wheelIyy := 0.5 * wm * sq(r)

	// Stripe geometry scales linearly with wheel radius from reference values.
	stripeOffset := roundTo4(r * simconfig.StripeOffsetFactor)
	stripeDims := [3]float64{
		roundTo4(simconfig.StripeDimX * r / simconfig.StripeRefRadius),
		roundTo4(simconfig.StripeDimY * r / simconfig.StripeRefRadius),
		roundTo4(simconfig.StripeDimZ * r / simconfig.StripeRefRadius),
	}

	link := parent.Sub("link", "name", name)
	link.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(poseText)

	// Main wheel cylinder (dark grey).
	vis := link.Sub("visual", "name", "visual")
	vis.SubT("pose", wheelRollPose)
	cyl := vis.Sub("geometry").Sub("cylinder")
	cyl.SubT("radius", ff(r))
	cyl.SubT("length", ff(w))
	setMaterial(vis, simconfig.RobotWheelColor)

	// Yellow rotation stripe — one quarter-turn offset so rotation is visible.
	sv := link.Sub("visual", "name", "stripe")
	sv.SubT("pose", pose6(stripeOffset, 0, 0, math.Pi/2, 0, 0))
	sv.Sub("geometry").Sub("box").SubT("size", vec3(stripeDims[0], stripeDims[1], stripeDims[2]))
	setMaterial(sv, simconfig.RobotWheelStripeColor)

	// Collision cylinder with ODE contact params.
	col := link.Sub("collision", "name", "collision")
	col.SubT("pose", wheelRollPose)
	cc := col.Sub("geometry").Sub("cylinder")
	cc.SubT("radius", ff(r))
	cc.SubT("length", ff(w))
	surf := col.Sub("surface")
	odeF := surf.Sub("friction").Sub("ode")
	odeF.SubT("mu", ff(simconfig.RobotWheelFrictionMu))
	odeF.SubT("mu2", ff(simconfig.RobotWheelFrictionMu))
	odeC := surf.Sub("contact").Sub("ode")
	odeC.SubT("kp", ff(simconfig.RobotWheelContactKp))
	odeC.SubT("kd", ff(simconfig.RobotWheelContactKd))
	odeC.SubT("max_vel", ff(simconfig.RobotWheelContactMaxVel))
	odeC.SubT("min_depth", ff(simconfig.RobotWheelContactMinDepth))

	iner := link.Sub("inertial")
	iner.SubT("mass", ff(wm))
	ix := iner.Sub("inertia")
	ix.SubT("ixx", fmt.Sprintf("%.8f", wheelIxx))
	ix.SubT("iyy", fmt.Sprintf("%.8f", wheelIyy))
	ix.SubT("izz", fmt.Sprintf("%.8f", wheelIxx))
}

func buildRearWheels(robot *Node) {
	halfWB := simconfig.RobotWheelbase / 2
	halfTrack := simconfig.RobotTrackWidth / 2

	for _, side := range []struct {
		name  string
		ySign float64
	}{
		{"rear_left_wheel", 1},
		{"rear_right_wheel", -1},
	} {
		buildWheelLink(robot, side.name,
			fmt.Sprintf("%s %s 0 0 0 0", ff(-halfWB), ff(side.ySign*halfTrack)))
	}

	for _, jt := range []struct {
		jointName, childName string
	}{
		{"rear_left_wheel_joint", "rear_left_wheel"},
		{"rear_right_wheel_joint", "rear_right_wheel"},
	} {
		j := robot.Sub("joint", "name", jt.jointName, "type", "revolute")
		j.SubT("parent", simconfig.RobotBaseFrameID)
		j.SubT("child", jt.childName)
		ax := j.Sub("axis")
		ax.SubT("xyz", "0 1 0")
		lim := ax.Sub("limit")
		lim.SubT("lower", "-1e16")
		lim.SubT("upper", "1e16")
		lim.SubT("effort", ff(simconfig.RobotRearJointEffort))
		lim.SubT("velocity", ff(simconfig.RobotRearJointVelocity))
		dyn := ax.Sub("dynamics")
		dyn.SubT("friction", ff(simconfig.RobotWheelJointFriction))
		dyn.SubT("damping", ff(simconfig.RobotWheelJointDamping))
	}
}

func buildFrontSteering(robot *Node) {
	halfWB := simconfig.RobotWheelbase / 2
	halfTrack := simconfig.RobotTrackWidth / 2

	for _, side := range []struct {
		name  string
		ySign float64
	}{
		{"left", 1},
		{"right", -1},
	} {
		yPos := side.ySign * halfTrack
		steerName := fmt.Sprintf("front_%s_steering", side.name)
		poseText := fmt.Sprintf("%s %s 0 0 0 0", ff(halfWB), ff(yPos))

		// Near-zero-inertia hinge link for the steering pivot.
		steerLink := robot.Sub("link", "name", steerName)
		steerLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(poseText)
		si := steerLink.Sub("inertial")
		si.SubT("mass", ff(simconfig.RobotSteeringLinkMass))
		six := si.Sub("inertia")
		for _, tag := range []string{simconfig.InertiaComponentIxx, simconfig.InertiaComponentIyy, simconfig.InertiaComponentIzz} {
			six.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
		}
		for _, tag := range []string{simconfig.InertiaComponentIxy, simconfig.InertiaComponentIxz, simconfig.InertiaComponentIyz} {
			six.SubT(tag, "0")
		}

		// Steering revolute joint (Z-axis rotation).
		sj := robot.Sub("joint", "name", fmt.Sprintf("front_%s_steering_joint", side.name), "type", "revolute")
		sj.SubT("parent", simconfig.RobotBaseFrameID)
		sj.SubT("child", steerName)
		sax := sj.Sub("axis")
		sax.SubT("xyz", "0 0 1")
		slim := sax.Sub("limit")
		slim.SubT("lower", "-"+ff(simconfig.RobotMaxSteering))
		slim.SubT("upper", ff(simconfig.RobotMaxSteering))
		slim.SubT("effort", ff(simconfig.RobotSteeringEffort))
		slim.SubT("velocity", ff(simconfig.RobotSteeringVelocity))

		// Wheel attached to steering hinge.
		wheelName := fmt.Sprintf("front_%s_wheel", side.name)
		buildWheelLink(robot, wheelName, poseText)

		// Wheel roll joint (Y-axis, attached to steering hinge as parent).
		wj := robot.Sub("joint", "name", fmt.Sprintf("front_%s_wheel_joint", side.name), "type", "revolute")
		wj.SubT("parent", steerName)
		wj.SubT("child", wheelName)
		wax := wj.Sub("axis")
		wax.SubT("xyz", "0 1 0")
		wlim := wax.Sub("limit")
		wlim.SubT("lower", "-1e16")
		wlim.SubT("upper", "1e16")
		wlim.SubT("effort", ff(simconfig.RobotFrontWheelEffort))
		wlim.SubT("velocity", ff(simconfig.RobotFrontWheelVelocity))
	}
}

func buildCameraLink(robot *Node) {
	camZ := simconfig.RobotHeight
	camPose := fmt.Sprintf("%s 0 %s 0 0 0", ff(simconfig.RobotLength/2), ff(camZ))

	camLink := robot.Sub("link", "name", "camera_link")
	camLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(camPose)
	setNominalInertial(camLink, simconfig.RobotCameraLinkMass)

	sensor := camLink.Sub("sensor", "name", "camera", "type", "camera")
	sensor.SubT("pose", pose6(0, 0, 0, 0, 0, 0))
	sensor.SubT("update_rate", ff(simconfig.CameraUpdateRate))
	sensor.SubT("visualize", "false")
	sensor.SubT("topic", simconfig.RobotCameraTopic)
	sensor.SubT("always_on", "true")
	cam := sensor.Sub("camera")
	cam.SubT("horizontal_fov", ff(simconfig.CameraHFOV))
	img := cam.Sub("image")
	img.SubT("width", fmt.Sprintf("%d", simconfig.CameraWidth))
	img.SubT("height", fmt.Sprintf("%d", simconfig.CameraHeight))
	img.SubT("format", simconfig.CameraImageFormat)
	clip := cam.Sub("clip")
	clip.SubT("near", ff(simconfig.CameraNearClip))
	clip.SubT("far", ff(simconfig.CameraFarClip))

	jt := robot.Sub("joint", "name", "camera_joint", "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", "camera_link")
}

func buildLidarLink(robot *Node) {
	lidarZ := simconfig.RobotHeight + simconfig.RobotLidarZOffset

	lidarLink := robot.Sub("link", "name", "lidar_link")
	lidarLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(
		fmt.Sprintf("0 0 %s 0 0 0", ff(lidarZ)))
	setNominalInertial(lidarLink, simconfig.RobotLidarLinkMass)

	sensor := lidarLink.Sub("sensor", "name", "lidar", "type", "gpu_lidar")
	sensor.SubT("update_rate", ff(simconfig.LidarUpdateRate))
	sensor.SubT("visualize", "true")
	sensor.SubT("topic", simconfig.RobotLidarTopic)
	sensor.SubT("always_on", "true")
	lidar := sensor.Sub("lidar")
	horiz := lidar.Sub("scan").Sub("horizontal")
	horiz.SubT("samples", fmt.Sprintf("%d", simconfig.LidarSamples))
	horiz.SubT("resolution", ff(simconfig.LidarHorizResolution))
	horiz.SubT("min_angle", fmt.Sprintf("%.6f", -math.Pi))
	horiz.SubT("max_angle", fmt.Sprintf("%.6f", math.Pi))
	rng := lidar.Sub("range")
	rng.SubT("min", ff(simconfig.LidarSimMinRange))
	rng.SubT("max", ff(simconfig.LidarMaxRange))
	rng.SubT("resolution", ff(simconfig.LidarRangeResolution))
	noise := lidar.Sub("noise")
	noise.SubT("type", simconfig.NoiseTypeGaussian)
	noise.SubT("mean", "0.0")
	noise.SubT("stddev", ff(simconfig.LidarNoiseStddev))

	jt := robot.Sub("joint", "name", "lidar_joint", "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", "lidar_link")
}

func buildImuLink(robot *Node) {
	imuLink := robot.Sub("link", "name", "imu_link")
	imuLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(
		fmt.Sprintf("0 0 %s 0 0 0", ff(simconfig.RobotImuZOffset)))

	ii := imuLink.Sub("inertial")
	ii.SubT("mass", ff(simconfig.ImuMass))
	iix := ii.Sub("inertia")
	for _, tag := range []string{"ixx", "iyy", "izz"} {
		iix.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
	}
	for _, tag := range []string{"ixy", "ixz", "iyz"} {
		iix.SubT(tag, "0")
	}

	sz := simconfig.ImuSize
	vis := imuLink.Sub("visual", "name", "visual")
	vis.Sub("geometry").Sub("box").SubT("size", vec3(sz[0], sz[1], sz[2]))
	setMaterial(vis, simconfig.RobotImuColor)

	sensor := imuLink.Sub("sensor", "name", "imu", "type", "imu")
	sensor.SubT("always_on", "true")
	sensor.SubT("update_rate", ff(simconfig.ImuUpdateRate))
	sensor.SubT("topic", simconfig.RobotImuTopic)
	imu := sensor.Sub("imu")
	for _, ag := range []struct {
		tag   string
		noise float64
	}{
		{"angular_velocity", simconfig.ImuGyroNoise},
		{"linear_acceleration", simconfig.ImuAccelNoise},
	} {
		agNode := imu.Sub(ag.tag)
		for _, axis := range []string{"x", "y", "z"} {
			n := agNode.Sub(axis).Sub("noise", "type", simconfig.NoiseTypeGaussian)
			n.SubT("mean", "0.0")
			n.SubT("stddev", ff(ag.noise))
		}
	}

	jt := robot.Sub("joint", "name", "imu_joint", "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", "imu_link")
}

func buildDebugOverheadCamera(world *Node) {
	model := New("model", "name", simconfig.ModelDebugCameraName)
	model.SubT("static", "true")
	model.SubT("pose", pose6(
		simconfig.TrackCenterCoord, simconfig.TrackCenterCoord, simconfig.DebugCameraZ, 0, 0, 0,
	))
	sensor := model.Sub("link", "name", "link").Sub("sensor", "name", "camera", "type", "camera")
	sensor.SubT("update_rate", fmt.Sprintf("%d", simconfig.DebugCameraUpdateRate))
	sensor.SubT("visualize", "true")
	sensor.SubT("topic", simconfig.DebugCameraTopic)
	sensor.SubT("always_on", "true")
	cam := sensor.Sub("camera")
	cam.SubT("horizontal_fov", ff(simconfig.DebugCameraFOV))
	img := cam.Sub("image")
	img.SubT("width", fmt.Sprintf("%d", simconfig.DebugCameraWidth))
	img.SubT("height", fmt.Sprintf("%d", simconfig.DebugCameraHeight))
	img.SubT("format", simconfig.CameraImageFormat)
	clip := cam.Sub("clip")
	clip.SubT("near", ff(simconfig.DebugCameraNearClip))
	clip.SubT("far", ff(simconfig.DebugCameraFarClip))
	world.Add(model)
}

// setMaterial adds <ambient> and <diffuse> sub-elements to a visual node.
func setMaterial(vis *Node, color simconfig.RGB) {
	mat := vis.Sub("material")
	c := rgba(color)
	mat.SubT("ambient", c)
	mat.SubT("diffuse", c)
}

// setNominalInertial adds a near-zero inertial block — used for sensor links
// that have negligible mass and inertia in the simulation.
func setNominalInertial(link *Node, mass float64) {
	inertial := link.Sub("inertial")
	inertial.SubT("mass", ff(mass))
	ix := inertial.Sub("inertia")
	for _, tag := range []string{"ixx", "iyy", "izz"} {
		ix.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
	}
}

// sq returns x*x.
func sq(x float64) float64 { return x * x }

// roundTo4 rounds to 4 decimal places (matches Python's round(..., 4)).
func roundTo4(x float64) float64 {
	return math.Round(x*10000) / 10000
}
