package hwconfig_test

import (
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	"github.com/teamvoltimor/vtitan/src/go/pkg/boardsim"
)

// The shipped board_sim.toml loads, converts milliseconds to durations,
// and describes a valid link.
func TestBoardSim_ShippedTOML(t *testing.T) {
	t.Setenv(profile.EnvVar, "")

	opts, err := hwconfig.BoardSim(shippedRoot)
	if err != nil {
		t.Fatalf("BoardSim: %v", err)
	}
	if opts.Tick <= 0 {
		t.Errorf("Tick = %v, want positive", opts.Tick)
	}
	if opts.Link.ToHost.Latency <= 0 || opts.Link.ToBoard.Latency <= 0 {
		t.Errorf("Link = %+v, want the shipped nonzero latencies", opts.Link)
	}
	if err = opts.Link.Validate(); err != nil {
		t.Errorf("shipped link: %v", err)
	}
}

func TestBoardSim_NoConfigRootIsIdeal(t *testing.T) {
	t.Parallel()

	opts, err := hwconfig.BoardSim("")
	if err != nil {
		t.Fatalf("BoardSim: %v", err)
	}
	if opts.Tick != boardsim.DefaultTick || opts.Link != (boardsim.LinkConfig{}) {
		t.Errorf("Options = %+v, want DefaultTick and an ideal link", opts)
	}
}

// Every field reaches Options with its unit converted, and values the
// schema forbids are rejected, since the TOML decoder does not apply it.
func TestBoardSim_Values(t *testing.T) {
	const good = `loop_tick_ms = 2.5
seed = 7
[to_board]
latency_ms = 3.0
jitter_ms = 0.5
corrupt_rate = 0.01
loss_rate = 0.05
loss_burst = 2.0
stall_rate_hz = 0.5
stall_ms = 200.0
[to_host]
latency_ms = 4.0
jitter_ms = 1.5
corrupt_rate = 0.02
loss_rate = 0.0
loss_burst = 1.0
stall_rate_hz = 0.0
stall_ms = 0.0
`
	t.Setenv(profile.EnvVar, "")

	opts, err := hwconfig.BoardSim(writeBoardSim(t, good))
	if err != nil {
		t.Fatalf("BoardSim: %v", err)
	}
	want := boardsim.Options{
		Tick: 2500 * time.Microsecond,
		Link: boardsim.LinkConfig{
			ToBoard: boardsim.Direction{
				Latency:       3 * time.Millisecond,
				Jitter:        500 * time.Microsecond,
				CorruptRate:   0.01,
				LossRate:      0.05,
				LossBurst:     2,
				StallRate:     0.5,
				StallDuration: 200 * time.Millisecond,
			},
			ToHost: boardsim.Direction{
				Latency:     4 * time.Millisecond,
				Jitter:      1500 * time.Microsecond,
				CorruptRate: 0.02,
				LossBurst:   1,
			},
			Seed: 7,
		},
	}
	if opts != want {
		t.Errorf("Options = %+v, want %+v", opts, want)
	}

	for name, swap := range map[string][2]string{
		"zero tick":        {"loop_tick_ms = 2.5", "loop_tick_ms = 0.0"},
		"negative seed":    {"seed = 7", "seed = -1"},
		"negative latency": {"latency_ms = 3.0", "latency_ms = -1.0"},
		"negative jitter":  {"jitter_ms = 1.5", "jitter_ms = -1.5"},
		"rate above one":   {"corrupt_rate = 0.02", "corrupt_rate = 1.5"},
		"loss of one":      {"loss_rate = 0.05", "loss_rate = 1.0"},
		"burst below one":  {"loss_burst = 2.0", "loss_burst = 0.5"},
		"negative stall":   {"stall_rate_hz = 0.5", "stall_rate_hz = -0.5"},
	} {
		broken := strings.Replace(good, swap[0], swap[1], 1)
		if broken == good {
			t.Fatalf("%s: %q not found in the fixture", name, swap[0])
		}
		if _, err = hwconfig.BoardSim(writeBoardSim(t, broken)); err == nil {
			t.Errorf("%s: BoardSim accepted %q", name, swap[1])
		}
	}
}

// writeBoardSim writes body as board_sim.toml under a fresh config root.
func writeBoardSim(t *testing.T, body string) string {
	t.Helper()

	root := t.TempDir()
	path := filepath.Join(root, filepath.FromSlash(profile.DefaultBoardSimTOMLPath))
	if err := os.MkdirAll(filepath.Dir(path), 0o750); err != nil {
		t.Fatal(err)
	}
	if err := os.WriteFile(path, []byte(body), 0o600); err != nil {
		t.Fatal(err)
	}
	return root
}
