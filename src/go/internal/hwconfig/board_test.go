package hwconfig_test

import (
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
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

// The shipped [lease] loads with its units converted and is valid for
// picolink.
func TestActuationBoard_ShippedLease(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	board, err := hwconfig.ActuationBoard(shippedRoot)
	if err != nil {
		t.Fatalf("ActuationBoard: %v", err)
	}
	l := board.Lease
	if l.BlindDistanceM <= 0 || l.Min <= 0 || l.Max < l.Min || l.OnExpiry != boardlink.ExpiryStop {
		t.Errorf("Lease = %+v, want the shipped positive, ordered lease with on_expiry stop", l)
	}
	if h := board.LinkHealth; h.SpeedCapMPS <= 0 || h.Hold <= 0 || h.MarginFloor <= 0 {
		t.Errorf("LinkHealth = %+v, want the shipped positive cap, hold and margin floor", h)
	}
}

// Values the schema forbids are rejected, since the TOML decoder does not
// apply it; an omitted table is no lease.
func TestActuationBoard_LeaseValues(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	write := func(body string) string {
		root := t.TempDir()
		path := filepath.Join(root, filepath.FromSlash(profile.DefaultBoardTOMLPath))
		if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
			t.Fatal(err)
		}
		if err := os.WriteFile(path, []byte("kind = \"pico2\"\nserial_port = \"/dev/x\"\n"+body), 0o600); err != nil {
			t.Fatal(err)
		}
		return root
	}

	board, err := hwconfig.ActuationBoard(write(""))
	if err != nil || board.Lease != (hwconfig.BoardLease{}) {
		t.Errorf("no [lease]: Lease = %+v, err %v, want no lease", board.Lease, err)
	}
	board, err = hwconfig.ActuationBoard(write(
		"[lease]\nblind_distance_m = 0.2\nmin_ms = 100.0\nmax_ms = 300.0\non_expiry = \"hold\"\n"))
	want := hwconfig.BoardLease{
		BlindDistanceM: 0.2, Min: 100 * time.Millisecond, Max: 300 * time.Millisecond, OnExpiry: boardlink.ExpiryHold,
	}
	if err != nil || board.Lease != want {
		t.Errorf("Lease = %+v, err %v, want %+v", board.Lease, err, want)
	}
	for name, body := range map[string]string{
		"unknown action": "[lease]\nblind_distance_m = 0.2\nmin_ms = 100.0\nmax_ms = 300.0\non_expiry = \"brake\"\n",
		"max below min":  "[lease]\nblind_distance_m = 0.2\nmin_ms = 300.0\nmax_ms = 100.0\non_expiry = \"stop\"\n",
		"no min":         "[lease]\nblind_distance_m = 0.2\nmin_ms = 0.0\nmax_ms = 100.0\non_expiry = \"stop\"\n",
	} {
		if _, err = hwconfig.ActuationBoard(write(body)); err == nil {
			t.Errorf("%s: ActuationBoard error = nil, want an error", name)
		}
	}
}
