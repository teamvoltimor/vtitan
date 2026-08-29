// Package motor implements the BTS7960 motor control loop shared by
// cmd/motor-node (bench/dev, standalone) and cmd/pi-zero (the combined
// board binary): subscribe AckermannCmd, drive internal/driver/motor,
// publish MotorStatus, enforce the command-deadline watchdog. Extracted
// here once a second real caller (cmd/pi-zero) needed the exact same
// logic — duplicating it, including the safety-critical watchdog, was a
// real risk (a fix landing in one copy but not the other), not a
// speculative "might need this elsewhere" abstraction.
package motor
