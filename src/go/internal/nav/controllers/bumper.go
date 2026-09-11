package controllers

// BumperGapAhead converts a FORWARD sensor range into the gap from the
// front bumper, matching bumper.bumper_gap_ahead.
//
// Clearance thresholds are policy statements about the CHASSIS ("do not get
// within 10 cm of something"), but the LIDAR reports distance from itself,
// and the sensor is not at the chassis center. Comparing a threshold
// against a raw range therefore measures it from wherever the sensor is
// mounted, so moving the mount silently redefines every threshold.
// lidarToFrontBumperM is the sensor-to-front-bumper offset (m), matching
// RobotSpecs.LIDAR_TO_FRONT_BUMPER (profile.RobotConfig.LidarToFrontBumper()).
func BumperGapAhead(rangeM, lidarToFrontBumperM float64) float64 {
	return rangeM - lidarToFrontBumperM
}

// BumperGapBehind converts a REAR sensor range into the gap from the rear
// bumper, matching bumper.bumper_gap_behind. lidarToRearBumperM is the
// sensor-to-rear-bumper offset (m), matching RobotSpecs.LIDAR_TO_REAR_BUMPER
// (profile.RobotConfig.LidarToRearBumper()).
func BumperGapBehind(rangeM, lidarToRearBumperM float64) float64 {
	return rangeM - lidarToRearBumperM
}
