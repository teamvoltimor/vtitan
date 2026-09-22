// Package servo drives the steering servo: one hobby RC servo on the Pi's
// hardware PWM engine (50 Hz, /sys/class/pwm), ported from
// src/python/src/hardware/motors/servo/driver.py.
//
// Why hardware PWM and not a software pulse train: on the Zero, software
// PWM carries kernel scheduling jitter straight into the pulse width, and
// the servo visibly twitched while holding a fixed angle
// (adr:0076-drivetrain-and-steering-hardware). The PWM overlay must map the
// engine onto the servo pin; without it Connect fails rather than falling
// back.
//
// The driver speaks SERVO degrees. Converting the wheel angle an
// AckermannCmd carries (through the linkage ratio, offset trim and travel
// clamp) is the motor node's job - see internal/node/motor - exactly as the
// Python split between ackermann_motor_node.py and servo/driver.py.
//
// Like pkg/driver/motor, this is an actuator, so it does not implement the
// sensor-shaped driver.Driver[T]; it exposes Connect/SetAngle/Close.
package servo
