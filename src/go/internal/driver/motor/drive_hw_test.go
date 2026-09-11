//go:build hw && hwinteractive && linux

package motor

import (
	"bufio"
	"context"
	"fmt"
	"os"
	"strconv"
	"strings"
	"testing"
	"time"
)

// waitForAck blocks until the operator types "y"/"yes" (pass) or anything
// else (fail). Used by interactive hardware tests that need a human eyeball
// or physical manipulation the test harness cannot perform itself.
func waitForAck(prompt string) bool {
	fmt.Printf("\n%s\n  [type y/yes to PASS, anything else to FAIL]: ", prompt)
	line, err := bufio.NewReader(os.Stdin).ReadString('\n')
	if err != nil {
		return false
	}
	line = strings.TrimSpace(strings.ToLower(line))
	return line == "y" || line == "yes"
}

func readIntEnv(env string, def int) int {
	if v := os.Getenv(env); v != "" {
		if n, err := strconv.Atoi(v); err == nil {
			return n
		}
	}
	return def
}

// TestHW_Motor_Spin_Dynamic frees the drive wheel and spins it forward then
// reverse, requiring an operator to confirm the wheel actually turns and
// reverses. No encoder exists in the Go port yet (see drive.go), so motion
// truth comes from a human watching the wheel -- this is the dynamic
// counterpart to TestHW_Motor_GPIO_PWM, which only proves wiring/claiming.
//
// PASS -> operator eyeballs forward + reverse rotation matching commanded
//
//	direction at a plausible speed; SetSpeed/Close succeed.
//
// FAIL  -> motor API errors (connect/SetSpeed/Close) OR operator reports no
//
//	motion / wrong direction. Either means the BTS7960 drive path is
//	miswired or the bridge isn't actually turning the wheel.
func TestHW_Motor_Spin_Dynamic(t *testing.T) {
	cfg := DefaultConfig()
	cfg.REnLine = readIntEnv("MOTOR_REN_LINE", cfg.REnLine)
	cfg.LEnLine = readIntEnv("MOTOR_LEN_LINE", cfg.LEnLine)
	cfg.ReversePWMLine = readIntEnv("MOTOR_REVERSE_PWM_LINE", cfg.ReversePWMLine)

	d, err := New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: motor.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: motor.Connect: %v", err)
	}
	defer func() { _ = d.Close() }()

	const spinSpeed = 0.4 // 40% duty: visible but not dangerous on a bench
	hold := 2 * time.Second

	// Forward.
	if err := d.SetSpeed(context.Background(), spinSpeed); err != nil {
		t.Fatalf("HW FAIL: motor.SetSpeed(+%.1f): %v", spinSpeed, err)
	}
	fmt.Printf("\n>>> Motor spinning FORWARD at %.0f%% duty for %s. Watch the wheel.", spinSpeed*100, hold)
	time.Sleep(hold)
	if !waitForAck("Did the wheel spin FORWARD (consistently one direction)?") {
		t.Fatalf("HW FAIL: operator did not confirm forward rotation")
	}

	// Stop briefly so direction is unambiguous to the operator.
	if err := d.SetSpeed(context.Background(), 0); err != nil {
		t.Fatalf("HW FAIL: motor.SetSpeed(0): %v", err)
	}
	time.Sleep(500 * time.Millisecond)

	// Reverse.
	if err := d.SetSpeed(context.Background(), -spinSpeed); err != nil {
		t.Fatalf("HW FAIL: motor.SetSpeed(-%.1f): %v", -spinSpeed, err)
	}
	fmt.Printf("\n>>> Motor spinning REVERSE at %.0f%% duty for %s. Watch the wheel.", spinSpeed*100, hold)
	time.Sleep(hold)
	if !waitForAck("Did the wheel spin REVERSE (opposite direction)?") {
		t.Fatalf("HW FAIL: operator did not confirm reverse rotation")
	}

	if err := d.SetSpeed(context.Background(), 0); err != nil {
		t.Fatalf("HW FAIL: motor.SetSpeed(0) on cleanup: %v", err)
	}

	t.Logf("HW PASS: drive wheel spun forward + reverse per operator (REN=%d LEN=%d LPWM=%d)",
		cfg.REnLine, cfg.LEnLine, cfg.ReversePWMLine)
}

// TestHW_Servo_Sweep_Dynamic commands the steering servo across its range
// (-15/+15/center) via the kernel hardware PWM (sysfs), mirroring Python's
// servo.Driver (pwmchip0/pwm0, GPIO12, 50 Hz). The servo has no position
// feedback, so the test reports the commanded pulse/angle and asks the
// operator to confirm the linkage actually moves to each position. This is
// the Go port's first dynamic steering exercise -- there is no servo driver
// type yet, so it drives sysfsPWMChannel directly (the same primitive the
// motor Driver uses for RPWM).
//
// PASS -> operator confirms the wheels/linkage move to center, +15, -15 and
//
//	back, and the PWM channel exports/initializes without error.
//
// FAIL  -> sysfs PWM export/init errors (overlay missing) OR operator
//
//	reports no movement at a commanded position. Either means the
//	steering PWM path is dead or the servo isn't on pwmchip0/pwm0.
func TestHW_Servo_Sweep_Dynamic(t *testing.T) {
	const (
		servoChip    = 0
		servoChannel = 0
		freqHz       = 50
		rangeDeg     = 180.0
		minPulseUS   = 500.0
		maxPulseUS   = 2500.0
		centerPulse  = 1500.0
		servoGPIO    = 12
	)

	pwm := newSysfsPWMChannel(sysfsPWMRoot, servoChip, servoChannel, freqHz)
	if err := pwm.Export(context.Background()); err != nil {
		t.Fatalf("HW FAIL: servo PWM export (overlay dtoverlay=pwm,pin=%d,func=4?): %v", servoGPIO, err)
	}
	if err := pwm.Init(context.Background()); err != nil {
		t.Fatalf("HW FAIL: servo PWM init: %v", err)
	}
	defer func() { _ = pwm.Disable() }()

	// centerPulse/rangeDeg are the Python servo.config.py defaults; if the
	// real servo.toml differs, override via env.
	minUS := envFloat("SERVO_MIN_PULSE_US", minPulseUS)
	maxUS := envFloat("SERVO_MAX_PULSE_US", maxPulseUS)
	cntUS := envFloat("SERVO_CENTER_PULSE_US", centerPulse)
	rngDeg := envFloat("SERVO_RANGE_DEG", rangeDeg)
	periodNS := time.Second.Nanoseconds() / int64(freqHz)

	// angleToFraction maps a signed servo angle (deg, 0=center) to a [0,1]
	// duty fraction against the 50 Hz frame, exactly like the Python
	// _position_to_pulse_us conversion.
	angleToFraction := func(angleDeg float64) float64 {
		spanUS := maxUS - minUS
		pulseUS := cntUS + (angleDeg/rngDeg)*spanUS
		pulseUS = max(minUS, min(maxUS, pulseUS))
		return pulseUS * 1000.0 / float64(periodNS)
	}

	steps := []struct {
		label string
		angle float64
	}{
		{"CENTER (0 deg)", 0},
		{"RIGHT (+15 deg)", 15},
		{"LEFT (-15 deg)", -15},
		{"CENTER (0 deg)", 0},
	}

	for _, s := range steps {
		frac := angleToFraction(s.angle)
		if err := pwm.SetDuty(context.Background(), frac); err != nil {
			t.Fatalf("HW FAIL: servo SetDuty(%.4f) at %s: %v", frac, s.label, err)
		}
		time.Sleep(1 * time.Second) // let the servo settle to the new position
		if !waitForAck(fmt.Sprintf("Servo sweep -> %s. Did the steering linkage move there?", s.label)) {
			t.Fatalf("HW FAIL: operator did not confirm servo at %s", s.label)
		}
	}

	t.Logf("HW PASS: servo swept CENTER/LEFT/RIGHT per operator (pwmchip%d/pwm%d GPIO%d)",
		servoChip, servoChannel, servoGPIO)
}

func envFloat(env string, def float64) float64 {
	if v := os.Getenv(env); v != "" {
		if f, err := strconv.ParseFloat(v, 64); err == nil {
			return f
		}
	}
	return def
}
