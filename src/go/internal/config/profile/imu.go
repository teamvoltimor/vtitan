package profile

// DefaultIMUUARTRVCTOMLPath is
// src/config/hardware/imu/bno08x_uart_rvc.toml, relative to the
// repo root.
//
// The file's shape is the generated imu.HardwareImuBno08XUartRvc DTO, whose
// [quaternion] section is imu.HardwareImuBno08XUartRvcQuaternion. Only
// DefaultPort and Baudrate have an internal/driver/imu.Config counterpart
// (Port, BaudRate) today; PollRateHz/SerialTimeout/DataLockTimeout/Quaternion
// describe driver-internal timing and axis convention the Go port doesn't
// parameterize yet.
const DefaultIMUUARTRVCTOMLPath = "src/config/hardware/imu/bno08x_uart_rvc.toml"
