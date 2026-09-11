//go:build hw

package imu_test

import (
	"context"
	"os"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/imu"
)

// TestHW_IMU_UART verifies go.bug.st/serial enumerates and opens the real
// BNO08x UART-RVC port and a valid RVC frame streams within a bounded
// timeout.
//
// PASS -> serial.Open succeeds AND readFrame decodes >=1 well-formed RVC
//         frame (checksum valid, known header) from the live sensor.
// FAIL -> serial.Open errors (port missing / permission / wrong tty) OR no
//         valid frame arrives before the read timeout (sensor not streaming,
//         wrong baud, or wiring). Any of these means the go.bug.st/serial
//         UART path or the RVC frame parser is broken against real hardware.
func TestHW_IMU_UART(t *testing.T) {
	port := os.Getenv("IMU_TTY")
	if port == "" {
		port = "/dev/ttyACM0"
	}
	cfg := imu.Config{Port: port, BaudRate: imu.DefaultBaudRate}

	d, err := imu.New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: imu.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: imu.Connect (serial.Open %s): %v", port, err)
	}
	defer func() { _ = d.Close() }()

	ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
	defer cancel()

	reading, err := d.Read(ctx)
	if err != nil {
		t.Fatalf("HW FAIL: imu.Read (no valid RVC frame from %s): %v", port, err)
	}

	t.Logf("HW PASS: IMU UART streamed valid RVC frame from %s (yaw=%.3f pitch=%.3f roll=%.3f)",
		port, reading.Yaw, reading.Pitch, reading.Roll)
}
