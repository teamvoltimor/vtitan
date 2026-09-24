package hwconfig

import (
	"fmt"
	"path/filepath"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/pkg/boardsim"
)

// BoardSim resolves the virtual actuation board's Options from
// configRoot's board_sim.toml, overlaid with the profiles named in
// profile.ActiveNames(): the loop tick and the link emulation. The
// hardware-less fields (BootID, the wheel model, NoEncoder, NoButton) are
// left for the caller, which knows what it is simulating.
//
// An empty configRoot is the ideal board: boardsim.DefaultTick and a link
// with no delay or corruption. A load failure or an invalid value is an
// error rather than a fallback, because a simulation silently run on an
// ideal link would look better than the configured one.
func BoardSim(configRoot string) (boardsim.Options, error) {
	if configRoot == "" {
		return boardsim.Options{Tick: boardsim.DefaultTick}, nil
	}

	loaded, err := profile.Load[hardware.HardwareBoardSim](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultBoardSimTOMLPath)),
		profile.ActiveNames(),
	)
	if err != nil {
		return boardsim.Options{}, fmt.Errorf("board sim: loading board_sim.toml: %w", err)
	}
	if loaded.LoopTickMs <= 0 {
		return boardsim.Options{}, fmt.Errorf("board sim: loop_tick_ms %v must be positive", loaded.LoopTickMs)
	}
	if loaded.Seed < 0 {
		return boardsim.Options{}, fmt.Errorf("board sim: seed %d must not be negative", loaded.Seed)
	}

	opts := boardsim.Options{
		Tick: millis(loaded.LoopTickMs),
		Link: boardsim.LinkConfig{
			ToBoard: boardsim.Direction{
				Latency:     millis(loaded.ToBoard.LatencyMs),
				Jitter:      millis(loaded.ToBoard.JitterMs),
				CorruptRate: loaded.ToBoard.CorruptRate,
			},
			ToHost: boardsim.Direction{
				Latency:     millis(loaded.ToHost.LatencyMs),
				Jitter:      millis(loaded.ToHost.JitterMs),
				CorruptRate: loaded.ToHost.CorruptRate,
			},
			Seed: uint64(loaded.Seed),
		},
	}
	// The TOML decoder does not apply the schema's bounds, so check here.
	if err = opts.Link.Validate(); err != nil {
		return boardsim.Options{}, fmt.Errorf("board sim: %w", err)
	}
	return opts, nil
}

// millis converts a TOML millisecond value to a Duration.
func millis(ms float64) time.Duration {
	return time.Duration(ms * float64(time.Millisecond))
}
