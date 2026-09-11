// Package lidar implements driver.Driver[Scan] (see
// platform/robot-go/internal/driver) for the Slamtec RPLIDAR C1 — the
// LIDAR this robot uses (platform/robot/docs references "LIDAR Slamtec
// C1"). Two scan-mode drivers are provided:
//
//   - DenseSerialDriver (frame_dense.go/driver_dense.go): the Express Scan
//     "Dense Mode" capsuled protocol. Preferred — this is the mode the
//     robot's Python stack actually validated on hardware (sllidar_ros2's
//     default scan_mode="Standard" always routes through
//     drv->startScanExpress, never the classic command), while classic
//     mode's range decode was found reading 2-4x too large on this exact
//     C1 unit despite matching the protocol doc's generic formula.
//   - ClassicSerialDriver (frame_classic.go/driver_classic.go): the
//     classic SCAN command, kept available for comparison/fallback now
//     that its range accuracy on this hardware is in question. It remains
//     the simplest, best-documented mode across the whole RPLIDAR family
//     (the C1 datasheet's "Protocol Compatibility" section confirms it's
//     "compatible with ... the traditional sampling protocol (standard)
//     of the A Series products").
//
// There is no existing Python driver for this sensor in the repo to port
// faithfully — the current stack uses the vendor's own sllidar_ros2
// package, not custom code — so both frame_*.go files are grounded
// directly in Slamtec's published protocol documentation (cited in each
// file's package comment) rather than an in-repo reference implementation.
package lidar
