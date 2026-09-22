// Package encoder provides the quadrature-encoder driver bolted to the
// drive motor shaft. It ports
// src/python/src/hardware/motors/encoder/{driver,control}.py; the pure
// count->distance/RPM primitives the driver is built on (the decoder, the
// speed estimator, Odometry) live in pkg/portable/quadrature, shared with
// the Pico 2 firmware.
//
// The encoder is a plain sensor, independent of whichever H-bridge is
// turning the shaft it reads -- the same separation Python's base.py
// EncoderSensor draws, and the reason this is its own package rather than
// fields on pkg/driver/motor.Driver.
//
// Only the closed-loop PID half of Python's control.py is deliberately NOT
// ported: the Go motor node drives open-loop (see internal/node/motor's
// SpeedToNormalized), so a PID here would be dead code. The count
// conversions and the speed estimator ARE ported, because they are what
// feeds wheel odometry -- the gap that kept internal/nav/bayexit sim-only,
// since natsgw.Gateway had no odometry source (see quadrature.go and the
// joint_states publisher in internal/node/motor). Bay exit is odometry's
// only consumer today; the parking safety clamp reads LIDAR clearances and
// never needed this.
package encoder
