package picolink

import (
	"fmt"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// LeasePolicy sizes each command's lease (boardlink.Command.DeadlineUS):
// how long the board may keep acting on it without a newer one. The lease
// is the time to cover BlindDistanceM at the commanded speed, clamped to
// [Min, Max], so a fast car gets a short lease and a slow or stopped one a
// long one. The board's own CommandTimeoutMS watchdog still bounds it: a
// lease only ever ends a command earlier.
//
// The zero value is off: commands go without a lease, as in protocol
// version 1.
type LeasePolicy struct {
	// BlindDistanceM is how far the car may drive on one command.
	BlindDistanceM float64
	// Min and Max bound the lease. Min keeps link jitter from expiring a
	// lease between two commands on time; Max caps a slow car's.
	Min, Max time.Duration
	// OnExpiry is what the board does when a lease runs out.
	OnExpiry boardlink.Expiry
}

// maxStampLead is how far in the future a command's stamp may be before it
// is not trusted: nav and picolink share the Pi 5's clock, so a stamp ahead
// of now is a clock step, not a command from the future.
const maxStampLead = 10 * time.Millisecond

// LeasePolicyFor is board.toml's [lease] as a LeasePolicy.
func LeasePolicyFor(l hwconfig.BoardLease) LeasePolicy {
	return LeasePolicy{BlindDistanceM: l.BlindDistanceM, Min: l.Min, Max: l.Max, OnExpiry: l.OnExpiry}
}

// Enabled reports whether commands get a lease at all.
func (p LeasePolicy) Enabled() bool {
	return p.BlindDistanceM > 0
}

// Validate reports a policy the lease cannot be computed from.
func (p LeasePolicy) Validate() error {
	switch {
	case !p.Enabled():
		return nil
	case p.Min <= 0 || p.Max < p.Min:
		return fmt.Errorf("picolink: lease bounds [%v, %v] must be positive and ordered", p.Min, p.Max)
	case p.OnExpiry > boardlink.ExpiryHold:
		return fmt.Errorf("picolink: unknown lease expiry action %d", p.OnExpiry)
	}
	return nil
}

// For is the lease of a command at speedMPS.
func (p LeasePolicy) For(speedMPS float64) time.Duration {
	speed := math.Abs(speedMPS)
	if speed == 0 {
		return p.Max
	}
	lease := time.Duration(p.BlindDistanceM / speed * float64(time.Second))
	return min(max(lease, p.Min), p.Max)
}

// commandFor turns an AckermannCmd into a boardlink Command with its lease.
//
// The lease runs from when the command was decided, its stamp, when that is
// set and plausible, else from now, when picolink forwards it; a command
// held up anywhere on the way (NATS, a stalled link) spends its lease
// there. The deadline is on the board's clock, through the latest Ping/Pong
// offset, so no lease is set before the first Pong of a boot.
func (s *Session) commandFor(cmd *actuationv1.AckermannCmd, now time.Time) boardlink.Command {
	out := boardlink.Command{SpeedMPS: s.capSpeed(cmd.GetSpeed(), now), SteeringAngleRad: cmd.GetSteeringAngle()}
	clock, ok := s.Clock()
	if !s.lease.Enabled() || !ok {
		return out
	}
	decided := now
	if stamp := cmd.GetStamp(); stamp.IsValid() && stamp.GetSeconds() > 0 {
		if t := stamp.AsTime(); !t.After(now.Add(maxStampLead)) {
			decided = t
		}
	}
	expires := decided.Add(s.lease.For(float64(out.SpeedMPS)))
	boardUS := int64(s.hostMicros(expires)) + clock.OffsetUS
	// Deadline 0 means "no lease" on the wire: a deadline that falls at or
	// before the board's boot is already expired, so send the earliest
	// real one instead.
	out.DeadlineUS = uint64(max(boardUS, 1))
	out.OnExpiry = s.lease.OnExpiry
	return out
}
