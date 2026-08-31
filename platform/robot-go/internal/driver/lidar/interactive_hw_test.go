//go:build hw && hwinteractive

package lidar_test

import (
	"bufio"
	"context"
	"fmt"
	"math"
	"os"
	"strconv"
	"strings"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver"
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

// newInteractiveDriver builds the lidar.Config every interactive hw test
// shares (port, baud, yaw offset from the environment) and constructs
// either lidar.ClassicSerialDriver or lidar.DenseSerialDriver depending on
// LIDAR_SCAN_MODE ("classic", the default, or "dense") -- so the same
// bearing-scan logic below can validate either scan-mode implementation
// without duplicating it. See frame_dense.go's package comment for why
// Dense mode is the one expected to actually read correct ranges.
func newInteractiveDriver(t *testing.T) driver.Driver[lidar.Scan] {
	t.Helper()

	port := os.Getenv("LIDAR_TTY")
	if port == "" {
		port = "/dev/ttyUSB0"
	}
	cfg := lidar.Config{Port: port, BaudRate: lidar.DefaultBaudRate}
	if v := os.Getenv("LIDAR_YAW_OFFSET_DEG"); v != "" {
		if off, err := strconv.ParseFloat(v, 64); err == nil {
			cfg.YawOffsetDeg = off
		}
	}

	mode := strings.ToLower(os.Getenv("LIDAR_SCAN_MODE"))
	switch mode {
	case "", "classic":
		d, err := lidar.NewClassic(cfg)
		if err != nil {
			t.Fatalf("HW FAIL: lidar.NewClassic: %v", err)
		}
		return d
	case "dense":
		d, err := lidar.NewDense(cfg)
		if err != nil {
			t.Fatalf("HW FAIL: lidar.NewDense: %v", err)
		}
		return d
	default:
		t.Fatalf("HW FAIL: unknown LIDAR_SCAN_MODE %q, want \"classic\" or \"dense\"", mode)
		return nil
	}
}

// TestHW_LIDAR_Object_Dynamic streams a scan and reports the closest valid
// return in each of the 8 compass bearings (0/45/90/135/180/225/270/315 deg,
// 45 deg sectors). The operator places objects at those bearings and confirms
// each shows up in its correct bucket at a plausible range. Unlike
// TestHW_LIDAR_UART (which only proves a scan assembles), this proves the
// decoded angle/range are physically correct -- the dynamic counterpart.
// Set LIDAR_SCAN_MODE=dense to run this against DenseSerialDriver instead
// of the default ClassicSerialDriver.
//
// PASS -> a scan assembles, the 8 bearings report (no hard range gate, since
//         the room may be open), and the operator confirms each placed object
//         lands in the expected bearing at a sane range.
// FAIL  -> Connect/Read errors, OR the operator reports an object did NOT
//         appear in its bearing (decode angle is wrong / offset off).
func TestHW_LIDAR_Object_Dynamic(t *testing.T) {
	d := newInteractiveDriver(t)
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: lidar.Connect: %v", err)
	}
	defer func() { _ = d.Close() }()

	fmt.Println(">>> LIDAR 8-bearing scan. Place objects at the 8 compass bearings")
	fmt.Println("   (0/45/90/135/180/225/270/315 deg around the robot) and confirm.")
	if !waitForAck("Objects placed at the bearings, run the scan?") {
		t.Fatalf("HW FAIL: operator aborted lidar sector test")
	}

	ctx, cancel := context.WithTimeout(context.Background(), 5*time.Second)
	defer cancel()
	scan, err := d.Read(ctx)
	if err != nil {
		t.Fatalf("HW FAIL: lidar.Read: %v", err)
	}

	// Bucket the closest return into 8 compass bearings (45 deg sectors
	// centered on 0/45/.../315). Only filter true no-return (RangeM==0) and
	// out-of-spec (> MaxRangeM); do NOT drop sub-MinRangeM returns -- the
	// operator's objects can be closer than the datasheet's 0.05 m near
	// limit, and those are exactly what we want to see.
	const (
		bearingHalfWidth = 22.5 // deg; half of the 45 deg sector
		maxR             = lidar.MaxRangeM
	)
	bearings := []struct {
		name   string
		center float64 // radians
		best   *lidar.Point
	}{
		{"0", 0, nil},
		{"45", math.Pi / 4, nil},
		{"90", math.Pi / 2, nil},
		{"135", 3 * math.Pi / 4, nil},
		{"180", math.Pi, nil},
		{"225", -3 * math.Pi / 4, nil},
		{"270", -math.Pi / 2, nil},
		{"315", -math.Pi / 4, nil},
	}

	validCount := 0
	for i := range scan {
		pt := scan[i]
		if pt.RangeM <= 0 || pt.RangeM > maxR {
			continue
		}
		validCount++
		for b := range bearings {
			da := pt.AngleRad - bearings[b].center
			da = math.Mod(da+math.Pi, 2*math.Pi) - math.Pi // wrap to (-pi,pi]
			if math.Abs(da) <= bearingHalfWidth*math.Pi/180 {
				if bearings[b].best == nil || pt.RangeM < bearings[b].best.RangeM {
					p := pt
					bearings[b].best = &p
				}
				break
			}
		}
	}

	fmt.Printf(">>> scan: %d points, %d valid\n", len(scan), validCount)
	for _, b := range bearings {
		if b.best == nil {
			fmt.Printf("    bearing %s: <no return>\n", b.name)
		} else {
			fmt.Printf("    bearing %s: angle=%6.1f deg range=%6.3f m\n",
				b.name, b.best.AngleRad*180/math.Pi, b.best.RangeM)
		}
	}

	if !waitForAck("Did each object you placed show up in its correct bearing at a sane range?") {
		t.Fatalf("HW FAIL: operator did not confirm lidar bearing geometry")
	}

	t.Logf("HW PASS: lidar sector scan per operator (valid=%d)", validCount)
}
