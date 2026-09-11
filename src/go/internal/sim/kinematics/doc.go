// Package kinematics implements the counter-phase four-wheel-steer
// integrator the headless simulator uses to advance the car's pose,
// ported from platform/robot/src/simulation/kinematics.py.
//
// Both axles steer, in opposite directions and by the same amount --
// confirmed on the real chassis 2026-07-25. That is not the textbook
// front-steer bicycle model, and the difference is not subtle:
// counter-phase steering moves the instantaneous center of rotation from
// the rear axle to the chassis center, so the robot yaws roughly TWICE as
// fast as a front-steer car at the same steering angle (the shipped
// rear_steer_ratio is 1.0):
//
//	x   += v * cos(yaw) * dt
//	y   += v * sin(yaw) * dt
//	yaw += (v / L_eff) * tan(steer) * dt   // L_eff = wheelbase / (1 + rearSteerRatio)
//
// With rearSteerRatio = 1.0 that is wheelbase / 2. Modeling this as a
// front-steer car made the simulation turn half as sharply as the
// hardware, so any gain tuned against it was hotter on the real robot than
// in sim -- see
// platform/robot/tests/unit/test_kinematics_4ws.py, this package's Go test
// oracle, whose docstring documents the historical regression (fixed
// 8eb3c38e).
//
// Integration is sub-stepped for accuracy at the 20 Hz control rate.
//
// Note the reference point: with symmetric counter-steer the body rotates
// about its own center, so (X, Y) tracks the chassis center rather than
// the rear axle -- unlike a front-steer car, whose rear tracks inside the
// front's path.
package kinematics
