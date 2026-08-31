//go:build hw && hwinteractive

package lidar_test

import (
	"bufio"
	"context"
	"fmt"
	"math"
	"os"
	"strings"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/lidar"
)

func waitForAck(prompt string) bool {
	fmt.Printf("\n%s\n  [type y/yes to PASS, anything else to FAIL]: ", prompt)
	line, err := bufio.NewReader(os.Stdin).ReadString('\n')
	if err != nil {
		return false
	}
	line = strings.TrimSpace(strings.ToLower(line))
	return line == "y" || line == "yes"
}

// TestHW_LIDAR_Object_Dynamic streams scans and asks the operator to place an
// object in front of the lidar, then verifies a return appears at the
// expected angle (near 0 rad, straight ahead) and a plausible range. Unlike
// TestHW_LIDAR_UART (which only proves a scan assembles), this proves the
// decoded geometry is physically correct -- the dynamic counterpart.
//
// PASS -> with an object placed ~0.3-1.0m in front, a scan point appears at
//         a near-forward angle and the expected range; operator confirms the
//         object was where they put it.
// FAIL  -> Connect/Read errors, OR no return shows up at the commanded
//         location when the operator confirms the object is there. Either
//         means the angle/range decoding or the scan assembly is wrong.
func TestHW_LIDAR_Object_Dynamic(t *testing.T) {
	port := os.Getenv("LIDAR_TTY")
	if port == "" {
		port = "/dev/ttyUSB0"
	}
	cfg := lidar.Config{Port: port, BaudRate: lidar.DefaultBaudRate}

	d, err := lidar.New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: lidar.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: lidar.Connect: %v", err)
	}
	defer func() { _ = d.Close() }()

	// Expect the operator to put the object roughly straight ahead (0 rad).
	const (
		expectAngleRad = 0.0
		angleTolRad    = 0.35 // ~20 degrees
		minRangeM      = 0.1
		maxRangeM      = 2.0
	)

	fmt.Println(">>> Place an object ~0.3-1.0m directly in FRONT of the lidar (straight ahead), then confirm.")
	if !waitForAck("Object placed in front, ready to scan?") {
		t.Fatalf("HW FAIL: operator aborted lidar object test")
	}

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	scan, err := d.Read(ctx)
	if err != nil {
		t.Fatalf("HW FAIL: lidar.Read: %v", err)
	}

	// Find the closest point within the forward angular window.
	var best *lidar.Point
	for i := range scan {
		pt := scan[i]
		da := math.Abs(pt.AngleRad - expectAngleRad)
		if da > math.Pi {
			da = 2*math.Pi - da
		}
		if da <= angleTolRad && (best == nil || pt.RangeM < best.RangeM) {
			p := pt
			best = &p
		}
	}

	if best == nil {
		t.Fatalf("HW FAIL: no return within %.0f deg of forward despite object placed there",
			angleTolRad*180/math.Pi)
	}
	if best.RangeM < minRangeM || best.RangeM > maxRangeM {
		t.Fatalf("HW FAIL: forward return range %.3f m outside plausible %.2f-%.2f m (decoding wrong?)",
			best.RangeM, minRangeM, maxRangeM)
	}

	fmt.Printf(">>> Forward return: angle=%.2f rad (%.0f deg) range=%.3f m\n",
		best.AngleRad, best.AngleRad*180/math.Pi, best.RangeM)

	if !waitForAck("Did the reported range/angle match where you placed the object?") {
		t.Fatalf("HW FAIL: operator did not confirm lidar geometry")
	}

	t.Logf("HW PASS: lidar returned object at angle=%.2f rad range=%.3f m per operator",
		best.AngleRad, best.RangeM)
}
