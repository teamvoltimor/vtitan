// Package imu publishes BNO08x UART-RVC readings on the vtitan.sensor.v1.imu
// subject, shared by cmd/imu-node (bench/dev, standalone) and cmd/pi5 (the
// combined board binary). It was extracted from cmd/imu-node once a second
// real caller needed the identical loop, for the same reason
// internal/node/motor was extracted from cmd/motor-node -- see that package's
// doc.go.
//
// The split follows the same rule as everywhere else in this tree:
// pkg/driver/imu talks to the serial port and knows nothing about NATS,
// and this package owns the wire mapping (MessageFor) and the publish loop.
// The two binaries then differ only in how they build the Config, not in what
// they do with it.
package imu
