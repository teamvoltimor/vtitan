package profile_test

import (
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/config/profile"
)

func TestLoad_BTS7960Config(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.BTS7960Config](filepath.Join("testdata", "bts7960.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.ReversePWMPin != 26 {
		t.Errorf("ReversePWMPin = %v, want 26", cfg.ReversePWMPin)
	}
	if cfg.REnPin != 6 || cfg.LEnPin != 5 {
		t.Errorf("REnPin/LEnPin = %v/%v, want 6/5", cfg.REnPin, cfg.LEnPin)
	}
}

func TestLoad_LidarLaunchConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.LidarLaunchConfig](
		filepath.Join("testdata", "lidar_launch.toml"),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.SerialPort != "/dev/ttyUSB0" {
		t.Errorf("SerialPort = %q, want /dev/ttyUSB0", cfg.SerialPort)
	}
	if cfg.SerialBaudrate != 460800 {
		t.Errorf("SerialBaudrate = %v, want 460800", cfg.SerialBaudrate)
	}
}

func TestLoad_IMUUARTRVCConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.IMUUARTRVCConfig](
		filepath.Join("testdata", "bno08x_uart_rvc.toml"),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.DefaultPort != "/dev/ttyACM0" || cfg.Baudrate != 115200 {
		t.Errorf(
			"DefaultPort/Baudrate = %q/%v, want /dev/ttyACM0/115200",
			cfg.DefaultPort,
			cfg.Baudrate,
		)
	}
	if !cfg.Quaternion.NegateYaw || cfg.Quaternion.NegatePitch || !cfg.Quaternion.NegateRoll {
		t.Errorf(
			"Quaternion = %+v, want NegateYaw/NegateRoll true, NegatePitch false",
			cfg.Quaternion,
		)
	}
}

func TestLoad_ButtonGPIOConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.ButtonGPIOConfig](filepath.Join("testdata", "gpio.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.ButtonGPIOPin != 4 {
		t.Errorf("ButtonGPIOPin = %v, want 4", cfg.ButtonGPIOPin)
	}
	if !cfg.Button.PullUp || cfg.Button.LongPressThresholdSec != 3.0 ||
		cfg.Button.ShutdownPressThresholdSec != 10.0 {
		t.Errorf("Button = %+v, want PullUp=true, LongPress=3.0, Shutdown=10.0", cfg.Button)
	}
}

func TestLoad_ButtonNodeConfig(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.ButtonNodeConfig](
		filepath.Join("testdata", "button_node.toml"),
		nil,
	)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.PollHz != 20.0 {
		t.Errorf("PollHz = %v, want 20.0", cfg.PollHz)
	}
}

func TestLoad_SSD1306Config(t *testing.T) {
	t.Parallel()

	cfg, err := profile.Load[profile.SSD1306Config](filepath.Join("testdata", "ssd1306.toml"), nil)
	if err != nil {
		t.Fatalf("Load: %v", err)
	}
	if cfg.Width != 128 || cfg.Height != 64 || cfg.I2CBus != 1 {
		t.Errorf("Width/Height/I2CBus = %v/%v/%v, want 128/64/1", cfg.Width, cfg.Height, cfg.I2CBus)
	}

	addr, err := cfg.I2CAddress()
	if err != nil {
		t.Fatalf("I2CAddress: %v", err)
	}
	if addr != 0x3C {
		t.Errorf("I2CAddress() = %#x, want 0x3C", addr)
	}
}
