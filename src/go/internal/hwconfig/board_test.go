package hwconfig_test

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
)

// The shipped board.toml is the Zero; appending the pico2 profile to the
// servo and motor profiles selects the Pico, and must not stop those
// profiles' other files from loading.
func TestActuationBoard_ShippedTOML(t *testing.T) {
	cases := []struct {
		profiles string
		want     hardware.HardwareBoardKind
	}{
		{"", hardware.HardwareBoardKindZero},
		{"270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm", hardware.HardwareBoardKindZero},
		{"270deg-hiwonder-35kg,rev-hd-hex-motor-6000rpm,pico2", hardware.HardwareBoardKindPico2},
	}
	for _, tc := range cases {
		t.Run(tc.profiles, func(t *testing.T) {
			t.Setenv(profile.EnvVar, tc.profiles)

			board, err := hwconfig.ActuationBoard(shippedRoot)
			if err != nil {
				t.Fatalf("ActuationBoard: %v", err)
			}
			if board.Kind != tc.want {
				t.Errorf("Kind = %q, want %q", board.Kind, tc.want)
			}
			if board.SerialPort == "" {
				t.Error("SerialPort is empty, want the base file's port")
			}
			if _, err = hwconfig.Servo(shippedRoot); err != nil {
				t.Errorf("Servo with the same profiles: %v", err)
			}
		})
	}
}

func TestActuationBoard_NoConfigRootIsTheZero(t *testing.T) {
	t.Parallel()

	board, err := hwconfig.ActuationBoard("")
	if err != nil {
		t.Fatalf("ActuationBoard: %v", err)
	}
	if board.Kind != hardware.HardwareBoardKindZero {
		t.Errorf("Kind = %q, want %q", board.Kind, hardware.HardwareBoardKindZero)
	}
}

// The TOML decoder does not apply the schema's enum, so the loader must.
func TestActuationBoard_UnknownKindErrors(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	root := t.TempDir()
	path := filepath.Join(root, filepath.FromSlash(profile.DefaultBoardTOMLPath))
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte("kind = \"pi4\"\nserial_port = \"/dev/ttyACM0\"\n"), 0o600); err != nil {
		t.Fatal(err)
	}

	if _, err := hwconfig.ActuationBoard(root); err == nil {
		t.Fatal("ActuationBoard: want error for an unknown kind, got nil")
	}
}
