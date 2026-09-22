// Package stateestimator resolves state_estimator.toml's heading-fusion
// tuning, the yaw_correction_gain of the complementary filter that pulls the
// IMU heading toward the wall-derived one. It ports the config half of
// src/python/src/state_machine/estimator.py; the fusion itself is applied by
// the gateway that owns the heading (internal/adapters/natsgw), whose heading
// offset is the same accumulator Python's _yaw_correction is.
package stateestimator
