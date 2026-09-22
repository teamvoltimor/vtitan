// Package imu provides the Driver interface and UART-RVC/I2C implementations
// for the IMU sensor. The RVC frame parser and the Euler-to-quaternion
// conversion live in pkg/portable/bno085rvc, shared with the Pico 2
// firmware; this package keeps the serial-port driver and its config.
package imu
