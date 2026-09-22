package actuation

// DefaultSpeedScalePercentPerMPS converts a commanded AckermannCmd.speed
// [m/s] into a motor duty percentage, matching motors.toml's
// `drive.speed_scale` (motor_speed = velocity_m_s * scale) - see
// src/config/hardware/motors/motors.toml. A caller with real
// hardware-profile data should load motors.HardwareMotorsMotors instead
// (internal/config/profile) and pass its Drive.SpeedScale to the motor
// loop; this is the fallback for callers that don't.
const DefaultSpeedScalePercentPerMPS = 30.0

// MaxDutyPercent is motors.toml's `drive.max_speed`/`min_speed` magnitude:
// motor duty percentage is clamped to [-100, 100].
const MaxDutyPercent = 100.0

// SpeedToNormalized converts an AckermannCmd's speed [m/s] into the signed
// duty fraction [-1, 1] motor.Actuator.SetSpeed expects, per
// scalePercentPerMPS (see DefaultSpeedScalePercentPerMPS).
func SpeedToNormalized(speedMPS float32, scalePercentPerMPS float64) float64 {
	percent := float64(speedMPS) * scalePercentPerMPS
	clamped := min(max(percent, -MaxDutyPercent), MaxDutyPercent)
	return clamped / MaxDutyPercent
}
