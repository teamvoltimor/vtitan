//go:build hw && hwinteractive

package imu_test

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"strings"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/imu"
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

// TestHW_IMU_Rotate_Dynamic streams IMU yaw/pitch/roll and asks the operator
// to rotate the board by hand, then confirms the angles changed by a
// plausible amount (not just noise). Unlike TestHW_IMU_UART (which only
// proves a valid frame arrives), this proves the readings track real-world
// motion -- the dynamic counterpart.
//
// PASS -> operator rotates the board and yaw/pitch/roll delta exceeds a
//         plausibility threshold (board actually moved); Read keeps
//         returning valid frames.
// FAIL  -> no valid frame streams, OR the deltas stay within noise while the
//         operator confirms they rotated it. Either means the RVC parser is
//         reporting garbage or the sensor isn't tracking orientation.
func TestHW_IMU_Rotate_Dynamic(t *testing.T) {
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
		t.Fatalf("HW FAIL: imu.Connect: %v", err)
	}
	defer func() { _ = d.Close() }()

	read := func() imu.Reading {
		ctx, cancel := context.WithTimeout(context.Background(), 3*time.Second)
		defer cancel()
		r, err := d.Read(ctx)
		if err != nil {
			t.Fatalf("HW FAIL: imu.Read during dynamic test: %v", err)
		}
		return r
	}

	before := read()
	fmt.Printf("\n>>> Baseline: yaw=%.2f pitch=%.2f roll=%.2f\n", before.Yaw, before.Pitch, before.Roll)
	fmt.Println(">>> Now rotate the board by hand (e.g. yaw it ~45-90 degrees). You have 10s.")
	time.Sleep(10 * time.Second)
	after := read()

	dyaw := after.Yaw - before.Yaw
	dpitch := after.Pitch - before.Pitch
	droll := after.Roll - before.Roll
	fmt.Printf(">>> After:   yaw=%.2f pitch=%.2f roll=%.2f\n", after.Yaw, after.Pitch, after.Roll)
	fmt.Printf(">>> Delta:   dyaw=%.2f dpitch=%.2f droll=%.2f\n", dyaw, dpitch, droll)

	// Normalize dyaw to (-180,180] so a wrap-around doesn't read as ~360.
	if dyaw > 180 {
		dyaw -= 360
	}
	if dyaw < -180 {
		dyaw += 360
	}
	maxDelta := maxAbs(dyaw, dpitch, droll)

	const plausibilityDeg = 10.0
	if maxDelta < plausibilityDeg {
		t.Fatalf("HW FAIL: IMU readings barely moved (max delta %.2f deg) despite rotation -- sensor not tracking", maxDelta)
	}

	if !waitForAck(fmt.Sprintf("Did the board actually rotate and the angles track it (max delta %.1f deg)?", maxDelta)) {
		t.Fatalf("HW FAIL: operator did not confirm IMU tracked rotation")
	}

	t.Logf("HW PASS: IMU tracked rotation (dyaw=%.2f dpitch=%.2f droll=%.2f)", dyaw, dpitch, droll)
}

func maxAbs(vals ...float64) float64 {
	m := 0.0
	for _, v := range vals {
		a := v
		if a < 0 {
			a = -a
		}
		if a > m {
			m = a
		}
	}
	return m
}
