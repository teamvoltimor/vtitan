package hwconfig

import (
	"fmt"
	"path/filepath"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/config/generated/hardware"
	"github.com/teamvoltimor/vtitan/src/go/internal/config/profile"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// Board is the actuation board the active hardware profile selects
// (adr:0098-pico-actuation-board-and-portable-cores).
type Board struct {
	// Kind is which board drives the steering servo and the drive motor.
	Kind hardware.HardwareBoardKind
	// SerialPort is the Pico 2's serial device on the Pi 5. Meaningful only
	// when Kind is hardware.HardwareBoardKindPico2.
	SerialPort string
	// Lease is board.toml's [lease]: how long each forwarded command
	// holds. A zero BlindDistanceM is no lease.
	Lease BoardLease
	// LinkHealth is board.toml's [link_health]: the speed cap on a
	// degraded link. A zero SpeedCapMPS never caps.
	LinkHealth BoardLinkHealth
}

// BoardLinkHealth is board.toml's [link_health], in the units
// picolink.HealthPolicy takes.
type BoardLinkHealth struct {
	MarginFloor, Hold time.Duration
	SpeedCapMPS       float64
}

// BoardLease is board.toml's [lease], in the units picolink.LeasePolicy
// takes.
type BoardLease struct {
	BlindDistanceM float64
	Min, Max       time.Duration
	OnExpiry       boardlink.Expiry
}

// ActuationBoard resolves configRoot's board.toml overlaid with the profiles
// named in profile.ActiveNames(). An empty configRoot is the Pi Zero, the
// board every build had before the Pico, so a dev machine behaves as before.
//
// Unlike Motor it does not fall back on a load failure: the answer decides
// which process commands the actuators, and guessing wrong either leaves the
// car with no driver or with two. An unknown kind is an error too, since the
// TOML decoder does not apply the schema's enum.
func ActuationBoard(configRoot string) (Board, error) {
	if configRoot == "" {
		return Board{Kind: hardware.HardwareBoardKindZero}, nil
	}

	loaded, err := profile.Load[hardware.HardwareBoard](
		filepath.Join(configRoot, filepath.FromSlash(profile.DefaultBoardTOMLPath)),
		profile.ActiveNames(),
	)
	if err != nil {
		return Board{}, fmt.Errorf("board: loading board.toml: %w", err)
	}

	switch loaded.Kind {
	case hardware.HardwareBoardKindZero, hardware.HardwareBoardKindPico2:
		lease, leaseErr := boardLease(loaded.Lease)
		if leaseErr != nil {
			return Board{}, leaseErr
		}
		health, healthErr := boardLinkHealth(loaded.LinkHealth)
		if healthErr != nil {
			return Board{}, healthErr
		}
		return Board{Kind: loaded.Kind, SerialPort: loaded.SerialPort, Lease: lease, LinkHealth: health}, nil
	default:
		return Board{}, fmt.Errorf("board: board.toml kind %q is neither %q nor %q",
			loaded.Kind, hardware.HardwareBoardKindZero, hardware.HardwareBoardKindPico2)
	}
}

// boardLease converts [lease], rejecting what the schema forbids, since the
// TOML decoder does not apply it. An omitted table is no lease.
func boardLease(l hardware.HardwareBoardLease) (BoardLease, error) {
	if l.BlindDistanceM == 0 {
		return BoardLease{}, nil
	}
	expiry := map[hardware.HardwareBoardLeaseOnExpiry]boardlink.Expiry{
		hardware.HardwareBoardLeaseOnExpiryStopCenter: boardlink.ExpiryStopCenter,
		hardware.HardwareBoardLeaseOnExpiryStop:       boardlink.ExpiryStop,
		hardware.HardwareBoardLeaseOnExpiryHold:       boardlink.ExpiryHold,
	}
	onExpiry, ok := expiry[l.OnExpiry]
	switch {
	case !ok:
		return BoardLease{}, fmt.Errorf("board: lease on_expiry %q is not stop_center, stop or hold", l.OnExpiry)
	case l.BlindDistanceM < 0 || l.MinMs <= 0 || l.MaxMs < l.MinMs:
		return BoardLease{}, fmt.Errorf("board: lease %+v needs a positive distance and 0 < min_ms <= max_ms", l)
	}
	return BoardLease{
		BlindDistanceM: l.BlindDistanceM,
		Min:            millis(l.MinMs),
		Max:            millis(l.MaxMs),
		OnExpiry:       onExpiry,
	}, nil
}

// boardLinkHealth converts [link_health], rejecting what the schema
// forbids. An omitted table never caps.
func boardLinkHealth(h hardware.HardwareBoardLinkHealth) (BoardLinkHealth, error) {
	if h.SpeedCapMps == 0 {
		return BoardLinkHealth{}, nil
	}
	if h.SpeedCapMps < 0 || h.HoldMs <= 0 || h.MarginFloorMs < 0 {
		return BoardLinkHealth{}, fmt.Errorf("board: link_health %+v needs a positive cap and hold", h)
	}
	return BoardLinkHealth{
		MarginFloor: millis(h.MarginFloorMs),
		Hold:        millis(h.HoldMs),
		SpeedCapMPS: h.SpeedCapMps,
	}, nil
}
