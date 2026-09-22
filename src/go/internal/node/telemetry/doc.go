// Package telemetry runs the telemetry-summary aggregation loop: it caches
// the latest IMU and LIDAR messages other nodes publish, feeds them to
// internal/telemetry/diag.Aggregator, and publishes the resulting
// TelemetrySummary on vtitan.ui.v1.telemetry_summary for cmd/pi-zero's OLED
// to render.
//
// Shared by cmd/telemetry-node (bench/dev, standalone) and cmd/pi5 (the
// combined board binary), extracted for the same reason as
// internal/node/{imu,lidar,motor}: a second real caller needed the identical
// loop.
//
// Unlike the sensor nodes, this one subscribes rather than driving hardware,
// so its Run owns three goroutines under one errgroup: two subscription
// readers and the fixed-rate publisher.
package telemetry
