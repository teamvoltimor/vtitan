package profile

// IMUQuaternionConfig mirrors bno08x_uart_rvc.toml's [quaternion] section --
// axis convention, not yet consumed by internal/driver/imu (see
// IMUUARTRVCConfig's doc comment).
type IMUQuaternionConfig struct {
	EulerSequence string `mapstructure:"euler_sequence"`
	NegateYaw     bool   `mapstructure:"negate_yaw"`
	NegatePitch   bool   `mapstructure:"negate_pitch"`
	NegateRoll    bool   `mapstructure:"negate_roll"`
}

// IMUUARTRVCConfig mirrors bno08x_uart_rvc.toml
// (src/hardware/imu/bno08x/uart_rvc.py). Only DefaultPort and Baudrate have
// an internal/driver/imu.Config counterpart (Port, BaudRate) today;
// PollRateHz/SerialTimeout/DataLockTimeout/Quaternion describe driver-
// internal timing and axis convention the Go port doesn't parameterize yet.
type IMUUARTRVCConfig struct {
	// DefaultPort matches internal/driver/imu.Config.Port.
	DefaultPort string `mapstructure:"default_port"`
	// Baudrate matches internal/driver/imu.Config.BaudRate.
	Baudrate        int                 `mapstructure:"baudrate"`
	PollRateHz      float64             `mapstructure:"poll_rate_hz"`
	SerialTimeout   float64             `mapstructure:"serial_timeout"`
	DataLockTimeout float64             `mapstructure:"data_lock_timeout"`
	Quaternion      IMUQuaternionConfig `mapstructure:"quaternion"`
}

// DefaultIMUUARTRVCTOMLPath is
// src/config/hardware/imu/bno08x_uart_rvc.toml, relative to the
// repo root.
const DefaultIMUUARTRVCTOMLPath = "src/config/hardware/imu/bno08x_uart_rvc.toml"
