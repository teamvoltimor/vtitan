//go:build hw

package motor_test

import (
	"context"
	"os"
	"strconv"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/motor"
)

// TestHW_Motor_GPIO_PWM verifies the go-gpiocdev output/PWM enable paths on
// real hardware: Connect claims the RPWM sysfs-PWM channel, the LPWM GPIO
// line, and the R_EN/L_EN enable lines, then runs the safety-critical
// zero-before-enable sequencing. The test drives duty 0 (no wheel motion) so
// it validates wiring/claiming without spinning the motor.
//
// PASS -> Connect claims all real lines/channels and enables the bridge with
//
//	both PWM channels at zero; SetSpeed(0) and Close succeed.
//
// FAIL -> Connect errors (a GPIO line or pwmchip is already claimed by another
//
//	consumer, /sys/class/pwm overlay missing, or R_EN/L_EN pin wrong) OR
//	SetSpeed/Close error. Any of these means the go-gpiocdev
//	output/enable or sysfs-PWM path is miswired against the live board.
//
// Override pins via env (BCM offsets): MOTOR_REN_LINE, MOTOR_LEN_LINE,
// MOTOR_REVERSE_PWM_LINE. Defaults match the project's BTS7960/IBT-2 wiring.
func TestHW_Motor_GPIO_PWM(t *testing.T) {
	cfg := motor.DefaultConfig()
	overrideInt(&cfg.REnLine, "MOTOR_REN_LINE")
	overrideInt(&cfg.LEnLine, "MOTOR_LEN_LINE")
	overrideInt(&cfg.ReversePWMLine, "MOTOR_REVERSE_PWM_LINE")

	d, err := motor.New(cfg)
	if err != nil {
		t.Fatalf("HW FAIL: motor.New: %v", err)
	}
	if err := d.Connect(context.Background()); err != nil {
		t.Fatalf("HW FAIL: motor.Connect (claim RPWM/LPWM/R_EN/L_EN): %v", err)
	}

	// Duty 0 keeps the bridge enabled but both channels zeroed -- no motion,
	// but it exercises the full Controller path after the enable sequencing.
	if err := d.SetSpeed(context.Background(), 0); err != nil {
		_ = d.Close()
		t.Fatalf("HW FAIL: motor.SetSpeed(0): %v", err)
	}

	if err := d.Close(); err != nil {
		t.Fatalf("HW FAIL: motor.Close: %v", err)
	}

	t.Logf("HW PASS: motor Connect/SetSpeed(0)/Close OK (REN=%d LEN=%d LPWM=%d pwmchip%d/pwm%d)",
		cfg.REnLine, cfg.LEnLine, cfg.ReversePWMLine, cfg.PWMChip, cfg.PWMChannel)
}

func overrideInt(dst *int, env string) {
	v, err := strconv.Atoi(os.Getenv(env))
	if err == nil {
		*dst = v
	}
}
