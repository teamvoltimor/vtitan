package hwconfig_test

import (
	"os"
	"path/filepath"
	"reflect"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// The shipped nats_faults.toml loads and injects nothing: a default that
// invents faults would bias every run.
func TestNATSFaults_ShippedTOMLInjectsNothing(t *testing.T) {
	t.Parallel()

	faults, err := hwconfig.NATSFaults(shippedRoot, nil)
	if err != nil {
		t.Fatalf("NATSFaults: %v", err)
	}
	if len(faults.Subjects) != 0 {
		t.Errorf("Subjects = %v, want none shipped", faults.Subjects)
	}
}

func TestNATSFaults_NoConfigRootIsNoFaults(t *testing.T) {
	t.Parallel()

	faults, err := hwconfig.NATSFaults("", nil)
	if err != nil {
		t.Fatalf("NATSFaults: %v", err)
	}
	if !reflect.DeepEqual(faults, nats.Faults{}) {
		t.Errorf("Faults = %+v, want the zero plan", faults)
	}
}

// Every field reaches the plan with its unit converted, an omitted
// drop_burst takes the schema's default of 1, and values the schema forbids
// are rejected, since the TOML decoder does not apply it.
func TestNATSFaults_Values(t *testing.T) {
	t.Parallel()

	const good = `seed = 9
[[subjects]]
subject = "vtitan.sensor.v1.scan"
delay_ms = 40.0
jitter_ms = 5.0
drop_rate = 0.1
freeze_rate_hz = 0.5
freeze_ms = 300.0
reorder_rate = 0.02
noise_sigma = 0.01
noise_fields = ["ranges"]
[[subjects]]
subject = "vtitan.sensor.v1.imu"
drop_rate = 0.2
drop_burst = 4.0
`
	faults, err := hwconfig.NATSFaults(writeNATSFaults(t, good), nil)
	if err != nil {
		t.Fatalf("NATSFaults: %v", err)
	}
	want := nats.Faults{
		Seed: 9,
		Subjects: map[string]nats.SubjectFaults{
			"vtitan.sensor.v1.scan": {
				NoiseFields:    []string{"ranges"},
				Delay:          40 * time.Millisecond,
				Jitter:         5 * time.Millisecond,
				FreezeDuration: 300 * time.Millisecond,
				DropRate:       0.1,
				DropBurst:      1,
				FreezeRate:     0.5,
				ReorderRate:    0.02,
				NoiseSigma:     0.01,
			},
			"vtitan.sensor.v1.imu": {DropRate: 0.2, DropBurst: 4},
		},
	}
	if !reflect.DeepEqual(faults, want) {
		t.Errorf("Faults =\n%+v\nwant\n%+v", faults, want)
	}

	bad := map[string]string{
		"wildcard subject":  "seed = 1\n[[subjects]]\nsubject = \"vtitan.>\"\n",
		"duplicate subject": "seed = 1\n[[subjects]]\nsubject = \"a.b\"\n[[subjects]]\nsubject = \"a.b\"\n",
		"drop rate of one":  "seed = 1\n[[subjects]]\nsubject = \"a.b\"\ndrop_rate = 1.0\n",
		"negative delay":    "seed = 1\n[[subjects]]\nsubject = \"a.b\"\ndelay_ms = -1.0\n",
		"negative seed":     "seed = -1\nsubjects = []\n",
	}
	for name, body := range bad {
		if _, err = hwconfig.NATSFaults(writeNATSFaults(t, body), nil); err == nil {
			t.Errorf("%s: NATSFaults error = nil, want an error", name)
		}
	}
}

func writeNATSFaults(t *testing.T, body string) string {
	t.Helper()

	root := t.TempDir()
	path := filepath.Join(root, filepath.FromSlash(profile.DefaultNATSFaultsTOMLPath))
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	return root
}
