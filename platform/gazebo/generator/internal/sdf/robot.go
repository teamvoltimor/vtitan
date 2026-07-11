package sdf

import (
	"fmt"
	"math"

	"voldemorbot/gazebo/generator/internal/simconfig"
)

var (
	// wheelRollPose is the 6-DOF pose that rotates a cylinder 90° around X to align
	// its axis with Y (the wheel roll axis in Gazebo).
	wheelRollPose = pose6(0, 0, 0, math.Pi/2, 0, 0)
)

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
	ix.SubT(simconfig.InertiaComponentIxx, fmt.Sprintf("%.6f", ixx))
	ix.SubT(simconfig.InertiaComponentIyy, fmt.Sprintf("%.6f", iyy))
	ix.SubT(simconfig.InertiaComponentIzz, fmt.Sprintf("%.6f", izz))
}

func buildAckermannPlugin(robot *Node) {
	plugin := robot.Sub("plugin",
		"filename", simconfig.PluginAckermannFilename,
		"name", simconfig.PluginAckermannName,
	)
	params := [][2]string{
		{"left_joint", simconfig.RobotJointRearLeft},
		{"right_joint", simconfig.RobotJointRearRight},
		{"left_steering_joint", simconfig.RobotJointFrontLeftSteer},
		{"right_steering_joint", simconfig.RobotJointFrontRightSteer},
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
	ix.SubT(simconfig.InertiaComponentIxx, fmt.Sprintf("%.8f", wheelIxx))
	ix.SubT(simconfig.InertiaComponentIyy, fmt.Sprintf("%.8f", wheelIyy))
	ix.SubT(simconfig.InertiaComponentIzz, fmt.Sprintf("%.8f", wheelIxx))
}

func buildRearWheels(robot *Node) {
	halfWB := simconfig.RobotWheelbase / 2
	halfTrack := simconfig.RobotTrackWidth / 2

	for _, side := range []struct {
		linkName string
		ySign    float64
	}{
		{simconfig.RobotLinkRearLeftWheel, 1},
		{simconfig.RobotLinkRearRightWheel, -1},
	} {
		buildWheelLink(robot, side.linkName, poseXY(-halfWB, side.ySign*halfTrack))
	}

	for _, jt := range []struct {
		jointName, childName string
	}{
		{simconfig.RobotJointRearLeft, simconfig.RobotLinkRearLeftWheel},
		{simconfig.RobotJointRearRight, simconfig.RobotLinkRearRightWheel},
	} {
		j := robot.Sub("joint", "name", jt.jointName, "type", "revolute")
		j.SubT("parent", simconfig.RobotBaseFrameID)
		j.SubT("child", jt.childName)
		ax := j.Sub("axis")
		ax.SubT("xyz", axisY)
		lim := ax.Sub("limit")
		lim.SubT("lower", jointLimitUnboundedNeg)
		lim.SubT("upper", jointLimitUnbounded)
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
		steerLink  string
		steerJoint string
		wheelLink  string
		wheelJoint string
		ySign      float64
	}{
		{
			steerLink:  simconfig.RobotLinkFrontLeftSteer,
			steerJoint: simconfig.RobotJointFrontLeftSteer,
			wheelLink:  simconfig.RobotLinkFrontLeftWheel,
			wheelJoint: simconfig.RobotJointFrontLeftWheel,
			ySign:      1,
		},
		{
			steerLink:  simconfig.RobotLinkFrontRightSteer,
			steerJoint: simconfig.RobotJointFrontRightSteer,
			wheelLink:  simconfig.RobotLinkFrontRightWheel,
			wheelJoint: simconfig.RobotJointFrontRightWheel,
			ySign:      -1,
		},
	} {
		yPos := side.ySign * halfTrack
		poseText := poseXY(halfWB, yPos)

		// Near-zero-inertia hinge link for the steering pivot.
		steerLink := robot.Sub("link", "name", side.steerLink)
		steerLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(poseText)
		si := steerLink.Sub("inertial")
		si.SubT("mass", ff(simconfig.RobotSteeringLinkMass))
		six := si.Sub("inertia")
		for _, tag := range []string{
			simconfig.InertiaComponentIxx,
			simconfig.InertiaComponentIyy,
			simconfig.InertiaComponentIzz,
		} {
			six.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
		}
		for _, tag := range []string{
			simconfig.InertiaComponentIxy,
			simconfig.InertiaComponentIxz,
			simconfig.InertiaComponentIyz,
		} {
			six.SubT(tag, intZero)
		}

		// Steering revolute joint (Z-axis rotation).
		sj := robot.Sub("joint", "name", side.steerJoint, "type", "revolute")
		sj.SubT("parent", simconfig.RobotBaseFrameID)
		sj.SubT("child", side.steerLink)
		sax := sj.Sub("axis")
		sax.SubT("xyz", axisZ)
		slim := sax.Sub("limit")
		slim.SubT("lower", "-"+ff(simconfig.RobotMaxSteering))
		slim.SubT("upper", ff(simconfig.RobotMaxSteering))
		slim.SubT("effort", ff(simconfig.RobotSteeringEffort))
		slim.SubT("velocity", ff(simconfig.RobotSteeringVelocity))

		// Wheel attached to steering hinge.
		buildWheelLink(robot, side.wheelLink, poseText)

		// Wheel roll joint (Y-axis, attached to steering hinge as parent).
		wj := robot.Sub("joint", "name", side.wheelJoint, "type", "revolute")
		wj.SubT("parent", side.steerLink)
		wj.SubT("child", side.wheelLink)
		wax := wj.Sub("axis")
		wax.SubT("xyz", axisY)
		wlim := wax.Sub("limit")
		wlim.SubT("lower", jointLimitUnboundedNeg)
		wlim.SubT("upper", jointLimitUnbounded)
		wlim.SubT("effort", ff(simconfig.RobotFrontWheelEffort))
		wlim.SubT("velocity", ff(simconfig.RobotFrontWheelVelocity))
	}
}

func buildCameraLink(robot *Node) {
	camPose := pose6(
		simconfig.RobotCameraMountXOffset, 0, simconfig.RobotCameraMountZOffset,
		0, simconfig.RobotCameraPitchRad, 0,
	)

	camLink := robot.Sub("link", "name", simconfig.RobotLinkCamera)
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
	img.SubT("width", fi(simconfig.CameraWidth))
	img.SubT("height", fi(simconfig.CameraHeight))
	img.SubT("format", simconfig.CameraImageFormat)
	clip := cam.Sub("clip")
	clip.SubT("near", ff(simconfig.CameraNearClip))
	clip.SubT("far", ff(simconfig.CameraFarClip))

	jt := robot.Sub("joint", "name", simconfig.RobotJointCamera, "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", simconfig.RobotLinkCamera)
}

func buildLidarLink(robot *Node) {
	lidarZ := simconfig.RobotHeight + simconfig.RobotLidarZOffset

	lidarLink := robot.Sub("link", "name", simconfig.RobotLinkLidar)
	lidarLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(
		pose6(simconfig.RobotLidarMountXOffset, 0, lidarZ, 0, 0, 0))
	setNominalInertial(lidarLink, simconfig.RobotLidarLinkMass)

	sensor := lidarLink.Sub("sensor", "name", "lidar", "type", "gpu_lidar")
	sensor.SubT("update_rate", ff(simconfig.LidarUpdateRate))
	sensor.SubT("visualize", "true")
	sensor.SubT("topic", simconfig.RobotLidarTopic)
	sensor.SubT("always_on", "true")
	lidar := sensor.Sub("lidar")
	horiz := lidar.Sub("scan").Sub("horizontal")
	horiz.SubT("samples", fi(simconfig.LidarSamples))
	horiz.SubT("resolution", ff(simconfig.LidarHorizResolution))
	horiz.SubT("min_angle", fmt.Sprintf("%.6f", simconfig.LidarMinAngle))
	horiz.SubT("max_angle", fmt.Sprintf("%.6f", simconfig.LidarMaxAngle))
	rng := lidar.Sub("range")
	rng.SubT("min", ff(simconfig.LidarSimMinRange))
	rng.SubT("max", ff(simconfig.LidarMaxRange))
	rng.SubT("resolution", ff(simconfig.LidarRangeResolution))
	noise := lidar.Sub("noise")
	noise.SubT("type", simconfig.NoiseTypeGaussian)
	noise.SubT("mean", floatZero)
	noise.SubT("stddev", ff(simconfig.LidarNoiseStddev))

	jt := robot.Sub("joint", "name", simconfig.RobotJointLidar, "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", simconfig.RobotLinkLidar)
}

func buildImuLink(robot *Node) {
	imuLink := robot.Sub("link", "name", simconfig.RobotLinkImu)
	imuLink.Sub("pose", "relative_to", simconfig.RobotBaseFrameID).T(
		fmt.Sprintf("0 0 %s 0 0 0", ff(simconfig.RobotImuZOffset)))

	ii := imuLink.Sub("inertial")
	ii.SubT("mass", ff(simconfig.ImuMass))
	iix := ii.Sub("inertia")
	for _, tag := range []string{
		simconfig.InertiaComponentIxx,
		simconfig.InertiaComponentIyy,
		simconfig.InertiaComponentIzz,
	} {
		iix.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
	}
	for _, tag := range []string{
		simconfig.InertiaComponentIxy,
		simconfig.InertiaComponentIxz,
		simconfig.InertiaComponentIyz,
	} {
		iix.SubT(tag, intZero)
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
			n.SubT("mean", floatZero)
			n.SubT("stddev", ff(ag.noise))
		}
	}

	jt := robot.Sub("joint", "name", simconfig.RobotJointImu, "type", "fixed")
	jt.SubT("parent", simconfig.RobotBaseFrameID)
	jt.SubT("child", simconfig.RobotLinkImu)
}

func buildDebugOverheadCamera(world *Node) {
	model := New("model", "name", simconfig.ModelDebugCameraName)
	model.SubT("static", "true")
	model.SubT("pose", pose6(
		simconfig.TrackCenterCoord, simconfig.TrackCenterCoord, simconfig.DebugCameraZ, 0, 0, 0,
	))
	sensor := model.Sub("link", "name", "link").Sub("sensor", "name", "camera", "type", "camera")
	sensor.SubT("update_rate", fi(simconfig.DebugCameraUpdateRate))
	sensor.SubT("visualize", "true")
	sensor.SubT("topic", simconfig.DebugCameraTopic)
	sensor.SubT("always_on", "true")
	cam := sensor.Sub("camera")
	cam.SubT("horizontal_fov", ff(simconfig.DebugCameraFOV))
	img := cam.Sub("image")
	img.SubT("width", fi(simconfig.DebugCameraWidth))
	img.SubT("height", fi(simconfig.DebugCameraHeight))
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
	for _, tag := range []string{
		simconfig.InertiaComponentIxx,
		simconfig.InertiaComponentIyy,
		simconfig.InertiaComponentIzz,
	} {
		ix.SubT(tag, ff(simconfig.RobotSteeringLinkInertia))
	}
}

// sq returns x*x.
func sq(x float64) float64 { return x * x }

// roundTo4 rounds to 4 decimal places (matches Python's round(..., 4)).
func roundTo4(x float64) float64 {
	return math.Round(x*10000) / 10000
}
