// Package lidar publishes RPLIDAR C1 scans on the vtitan.sensor.v1.scan
// subject, shared by cmd/lidar-node (bench/dev, standalone) and cmd/pi5 (the
// combined board binary). Extracted from cmd/lidar-node for the same reason
// internal/node/imu and internal/node/motor were: a second real caller needed
// the identical loop, and copying it would have let the bench binary and the
// board binary drift.
//
// pkg/driver/lidar owns the serial protocol and knows nothing about
// NATS; this package owns the LaserScan wire mapping (MessageFor) and the
// publish loop.
package lidar
