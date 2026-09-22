//go:build linux

package picolink_test

import (
	"log/slog"
	"os"
	"path/filepath"
	"strings"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	nodemotor "github.com/teamvoltimor/vtitan/src/go/internal/node/motor"
	"github.com/teamvoltimor/vtitan/src/go/internal/node/picolink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/encoder"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// shippedProfile is the profile the robot runs.
const shippedProfile = "270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm"

// shippedRoot is the repo root, from src/go/internal/node/picolink.
var shippedRoot = filepath.Join("..", "..", "..", "..", "..")

// Against the checked-in tree: the Config a Hello is answered with carries
// exactly what the Zero resolves from the same files. Each expected value
// names its source.
func TestHello_ConfigMatchesShippedProfile(t *testing.T) {
	t.Setenv(profile.EnvVar, shippedProfile)

	cfg, err := picolink.SessionConfigFor(slog.New(slog.DiscardHandler), shippedRoot, false,
		picolink.DefaultCommandTimeout)
	if err != nil {
		t.Fatalf("SessionConfigFor: %v", err)
	}
	want := boardlink.Config{
		CommandTimeoutMS:    500,                 // node/motor.DefaultCommandTimeout
		SpeedScalePctPerMPS: 30,                  // motors.toml drive.speed_scale
		InvertDrive:         false,               // the --pico-motor-invert flag
		LinkageRatio:        float32(85.0 / 135), // robot.toml via the servo profile
		ServoMaxAngleDeg:    135,                 // robot.toml steering.servo_max_angle_deg (profile)
		SteeringOffsetDeg:   0,                   // motors.toml steering.offset
		ServoMinPulseUS:     500,                 // servo.toml min_pulse_us
		ServoMaxPulseUS:     2500,                // servo.toml max_pulse_us
		ServoCenterPulseUS:  1500,                // servo.toml center_pulse_us
		ServoRangeDeg:       270,                 // servo.toml range_deg (profile overlay)
		ServoReversed:       false,               // servo.toml reversed
		HardwareWatchdogMS:  picolink.HardwareWatchdogMS,
		StatusIntervalMS:    picolink.StatusIntervalMS,
		OdometryIntervalMS:  picolink.OdometryIntervalMS,
	}
	if cfg.Board != want {
		t.Errorf("Board = %+v\nwant    %+v", cfg.Board, want)
	}
	// Inverted: motors.toml drive.encoder_reversed = true, which Python
	// applies and hwconfig.Encoder now does too (it used to be ignored).
	if cfg.Button == nil {
		t.Error("Button = nil, want the thresholds resolved from button.toml")
	}
	if cfg.Encoder == nil || cfg.Encoder.CountsPerRev != 60 || !cfg.Encoder.Invert {
		t.Errorf("Encoder = %+v, want counts_per_rev 60 from the motor profile, inverted by encoder_reversed",
			cfg.Encoder)
	}

	h := startSession(t, cfg)
	h.board.send(t, boardlink.Packet{Type: boardlink.TypeHello, Hello: boardlink.Hello{
		ProtocolVersion: boardlink.Version, BootID: 1,
	}})
	if got := h.board.expect(t, boardlink.TypeConfig); got.Config != want {
		t.Errorf("Config on the wire = %+v\nwant               %+v", got.Config, want)
	}
}

// With no encoder.toml the board still configures, and odometry is skipped
// with a warning, as cmd/pi-zero does. The tree is the shipped one with
// encoder.toml left out.
func TestSessionConfigFor_WithoutEncoder(t *testing.T) {
	t.Setenv(profile.EnvVar, shippedProfile)

	root := t.TempDir()
	for _, rel := range []string{
		profile.DefaultRobotTOMLPath,
		"src/config/profiles",
		profile.DefaultMotorsTOMLPath,
		profile.DefaultServoTOMLPath,
		"src/config/hardware/motors/profiles",
	} {
		target, err := filepath.Abs(filepath.Join(shippedRoot, rel))
		if err != nil {
			t.Fatal(err)
		}
		link := filepath.Join(root, rel)
		if err = os.MkdirAll(filepath.Dir(link), 0o750); err != nil {
			t.Fatal(err)
		}
		if err = os.Symlink(target, link); err != nil {
			t.Fatal(err)
		}
	}

	logs := &logBuffer{}
	cfg, err := picolink.SessionConfigFor(slog.New(slog.NewTextHandler(logs, nil)), root, false,
		picolink.DefaultCommandTimeout)
	if err != nil {
		t.Fatalf("SessionConfigFor: %v", err)
	}
	if cfg.Encoder != nil {
		t.Errorf("Encoder = %+v, want nil without a motor profile", cfg.Encoder)
	}
	if !strings.Contains(logs.String(), "no wheel encoder configured") {
		t.Errorf("no warning logged:\n%s", logs.String())
	}
}

// Without a servo profile there is no safe steering geometry, and without a
// config root nothing at all: both are errors, not guesses.
func TestSessionConfigFor_RefusesToGuess(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	logger := slog.New(slog.DiscardHandler)
	if _, err := picolink.SessionConfigFor(logger, shippedRoot, false, picolink.DefaultCommandTimeout); err == nil {
		t.Error("resolved a board config with no servo profile")
	}
	if _, err := picolink.SessionConfigFor(logger, "", false, picolink.DefaultCommandTimeout); err == nil {
		t.Error("resolved a board config with no config root")
	}
}

// The JointStates this package builds are the Zero's, field for field.
func TestJointStatesFor_MatchesTheZero(t *testing.T) {
	t.Parallel()

	const revolutions, rpm = 3.25, -142.5
	got := picolink.JointStatesFor(revolutions, rpm)
	want := nodemotor.JointStatesFor(encoder.Odometry{Revolutions: revolutions, RPM: rpm})
	if got.GetFrameId() != want.GetFrameId() || strings.Join(got.GetName(), ",") != strings.Join(want.GetName(), ",") ||
		got.GetPosition()[0] != want.GetPosition()[0] || got.GetVelocity()[0] != want.GetVelocity()[0] {
		t.Errorf("JointStatesFor = %v, the Zero's = %v", got, want)
	}
}

// The constants this package mirrors from the Zero still agree with it.
func TestMirroredConstants(t *testing.T) {
	t.Parallel()

	if picolink.FrameID != nodemotor.FrameID {
		t.Errorf("FrameID %q, the Zero's %q", picolink.FrameID, nodemotor.FrameID)
	}
	if picolink.OdometryIntervalMS != nodemotor.DefaultFeedbackInterval.Milliseconds() {
		t.Errorf("OdometryIntervalMS %d, the Zero's feedback interval %v",
			picolink.OdometryIntervalMS, nodemotor.DefaultFeedbackInterval)
	}
	if picolink.HardwareWatchdogMS >= nodemotor.DefaultCommandTimeout.Milliseconds() {
		t.Errorf("hardware watchdog %d ms is not shorter than the command timeout", picolink.HardwareWatchdogMS)
	}
}
