//go:build hw

package lidar_test

import (
	"context"
	"os"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/lidar"
)

// TestHW_LIDAR_UART verifies go.bug.st/serial enumerates and opens the real
// RPLIDAR C1 TTL UART, and that the device answers the SCAN request with a
// well-formed scan (>=1 decoded measurement).
//
// PASS -> serial.Open succeeds, the SCAN descriptor comes back as the
//         measurement data type, and Read assembles >=1 measurement point.
// FAIL -> serial.Open errors (port missing / permission / wrong tty) OR the
//         SCAN descriptor is wrong/missing (device not entering scan state,
//         wrong baud) OR no full scan assembles before timeout (protocol
//         parser broken against the live stream). Any of these means the
//         go.bug.st/serial UART path or the RPLIDAR frame parser is wrong
//         against real hardware.
func TestHW_LIDAR_UART(t *testing.T) {
	port := os.Getenv("LIDAR_TTY")
	if port == "" {
		port = "/dev/ttyAMA1"
	}
	cfg := lidar.Config{Port: port, BaudRate: lidar.DefaultBaudRate}

	d, err := lidar.New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: lidar.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: lidar.Connect (serial.Open %s + SCAN): %v", port, err)
	}
	defer func() { _ = d.Close() }()

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()

	scan, err := d.Read(ctx)
	if err != nil {
		t.Fatalf("HW FAIL: lidar.Read (no scan assembled from %s): %v", port, err)
	}
	if len(scan) == 0 {
		t.Fatalf("HW FAIL: lidar.Read returned an empty scan from %s", port)
	}

	t.Logf("HW PASS: LIDAR UART streamed a scan of %d points from %s", len(scan), port)
}
