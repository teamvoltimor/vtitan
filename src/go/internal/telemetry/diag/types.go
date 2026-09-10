package diag

// Detection is one vision-pipeline detection, as reported to the telemetry
// bridge over `/vision/detections`. Mirrors the fields of
// shared.domain.models.Detection that _best_detection (telemetry_bridge_node.py)
// actually reads: class name, confidence, and bounding-box width/height
// (used to derive area for ranking). There is no detection.proto in
// internal/schema/pb yet, so this stays a plain internal domain type until
// vision gets its own wire schema designed — that is a separate, deliberate
// step, not part of this pass.
type Detection struct {
	ClassName  string
	Confidence float64
	Width      float64
	Height     float64
}

// TelemetrySummary is the low-rate LIDAR/yaw/detection readout the OLED
// needs, field-for-field matching TelemetrySummaryWire
// (platform/robot/src/ros2/wire_models.py) and what
// telemetry_bridge_node.py's `_publish_ui_summary` computes:
//
//   - LidarFrontCM / LidarLeftCM / LidarRightCM: mean LIDAR range (cm) in
//     the front/left/right sectors, computed the same way
//     `_lidar_clearances` -> `clearances_from_scan` does (see sector.go).
//     0 when no scan has been received yet.
//   - GyroYawDeg: yaw extracted from the latest IMU orientation quaternion
//     (see quaternion.go). 0 when no IMU reading has been received yet.
//   - BestDetectionClassID / BestDetectionConfidence: the single
//     highest-(confidence*area) vision detection, matching
//     `_best_detection`. HasBestDetection distinguishes "no detection
//     available" from a zero-value detection, replacing the Python side's
//     `class_id, confidence = None, None` sentinel — Go has no natural
//     "optional field" equivalent for a plain struct.
type TelemetrySummary struct {
	BestDetectionClassID    string
	BestDetectionConfidence float64
	LidarFrontCM            float64
	LidarLeftCM             float64
	LidarRightCM            float64
	GyroYawDeg              float64
	HasBestDetection        bool
}

// Area returns the bounding-box area used to rank detections, matching
// shared.domain.models.Detection.area (width * height).
func (d Detection) Area() float64 {
	return d.Width * d.Height
}
