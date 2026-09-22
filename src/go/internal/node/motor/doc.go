// Package motor implements the actuator control loop shared by
// cmd/motor-node (bench/dev, standalone, drive only) and cmd/pi-zero (the
// combined board binary, drive and steering): subscribe AckermannCmd, drive
// pkg/driver/motor, steer pkg/driver/servo (steering.go converts the
// command's wheel angle to a servo angle), publish MotorStatus, and enforce
// the command-deadline watchdog, which stops the drive and centers the
// steering. Extracted
// here once a second real caller (cmd/pi-zero) needed the exact same
// logic — duplicating it, including the safety-critical watchdog, was a
// real risk (a fix landing in one copy but not the other), not a
// speculative "might need this elsewhere" abstraction.
package motor
