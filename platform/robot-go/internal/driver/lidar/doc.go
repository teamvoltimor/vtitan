// Package lidar implements driver.Driver[Scan] (see
// platform/robot-go/internal/driver) for the Slamtec RPLIDAR C1 — the
// LIDAR this robot uses (platform/robot/docs references "LIDAR Slamtec
// C1"). Only the classic SCAN command is implemented, not EXPRESS_SCAN or
// ULTRA modes: it's the simplest, most stable, best-documented mode across
// the entire RPLIDAR family including the C1 (the C1 datasheet's "Protocol
// Compatibility" section confirms it's "compatible with ... the
// traditional sampling protocol (standard) of the A Series products"), and
// matches this repo's bias toward simplicity over premature scope. There
// is no existing Python driver for this sensor in the repo to port
// faithfully — the current stack uses the vendor's own sllidar_ros2
// package, not custom code — so frame.go is grounded directly in Slamtec's
// published protocol documentation (cited in frame.go's package comment)
// rather than an in-repo reference implementation.
package lidar
