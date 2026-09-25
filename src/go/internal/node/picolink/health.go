package picolink

import (
	"fmt"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/hwconfig"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// HealthPolicy is how the host reacts to the link health the board reports
// in Status (platform plan item 2.14). A Status is evidence of a degraded
// link when a command arrived past its lease, a lease ran out, or a command
// arrived with less than MarginFloor of its lease left. From the last such
// evidence and for Hold, every command's speed is capped at SpeedCapMPS: a
// slower car drives less far blind, and its lease (LeasePolicy.For) is
// longer, so it is more likely to reach the board in time.
//
// The zero value never caps.
type HealthPolicy struct {
	MarginFloor time.Duration
	Hold        time.Duration
	SpeedCapMPS float64
}

// linkHealth is the host's running judgement of the link.
type linkHealth struct {
	degradedUntil time.Time
	// expired and expiries are the board's running totals at its previous
	// Status, valid when haveTotals; a new boot restarts them.
	expired, expiries uint16
	haveTotals        bool
	degraded          bool
}

// Enabled reports whether a degraded link caps speed at all.
func (p HealthPolicy) Enabled() bool {
	return p.SpeedCapMPS > 0
}

// Validate reports a policy that cannot work.
func (p HealthPolicy) Validate() error {
	if p.Enabled() && (p.Hold <= 0 || p.MarginFloor < 0) {
		return fmt.Errorf("picolink: link health hold %v must be positive and margin floor %v not negative",
			p.Hold, p.MarginFloor)
	}
	return nil
}

// HealthPolicyFor is board.toml's [link_health] as a HealthPolicy.
func HealthPolicyFor(h hwconfig.BoardLinkHealth) HealthPolicy {
	return HealthPolicy{MarginFloor: h.MarginFloor, Hold: h.Hold, SpeedCapMPS: h.SpeedCapMPS}
}

// observeHealth folds one Status into the judgement and returns the link
// health to publish with it.
func (s *Session) observeHealth(st boardlink.Status, now time.Time) *actuationv1.LinkHealth {
	h := &s.health
	reason := ""
	switch {
	case h.haveTotals && st.CommandsExpired != h.expired:
		reason = fmt.Sprintf("%d commands arrived past their lease", st.CommandsExpired-h.expired)
	case h.haveTotals && st.LeaseExpiries != h.expiries:
		reason = fmt.Sprintf("%d leases ran out", st.LeaseExpiries-h.expiries)
	case st.MinLeaseMarginMS != boardlink.NoLeaseMargin &&
		time.Duration(st.MinLeaseMarginMS)*time.Millisecond < s.healthPolicy.MarginFloor:
		reason = fmt.Sprintf("a command arrived with %d ms of lease left", st.MinLeaseMarginMS)
	}
	h.expired, h.expiries, h.haveTotals = st.CommandsExpired, st.LeaseExpiries, true

	if reason != "" && s.healthPolicy.Enabled() {
		if !h.degraded {
			s.logger.Warn("picolink: command link degraded, capping speed",
				"reason", reason, "speed_cap_mps", s.healthPolicy.SpeedCapMPS)
		}
		h.degraded, h.degradedUntil = true, now.Add(s.healthPolicy.Hold)
	}
	s.linkDegraded(now)

	out := &actuationv1.LinkHealth{
		MaxCommandGapMs:  uint32(st.MaxGapMS),
		HasLeaseMargin:   st.MinLeaseMarginMS != boardlink.NoLeaseMargin,
		CommandsExpired:  uint32(st.CommandsExpired),
		LeaseExpiries:    uint32(st.LeaseExpiries),
		Degraded:         h.degraded,
		SpeedCapMps:      float32(s.healthPolicy.SpeedCapMPS),
		MinLeaseMarginMs: 0,
	}
	if out.HasLeaseMargin {
		out.MinLeaseMarginMs = int32(st.MinLeaseMarginMS)
	}
	return out
}

// linkDegraded reports whether speed is capped now, ending a degraded
// spell once Hold has passed without new evidence.
func (s *Session) linkDegraded(now time.Time) bool {
	h := &s.health
	if h.degraded && now.After(h.degradedUntil) {
		h.degraded = false
		s.logger.Info("picolink: command link recovered, speed cap lifted")
	}
	return h.degraded
}

// capSpeed limits speed to the policy's cap while the link is degraded,
// keeping its sign.
func (s *Session) capSpeed(speed float32, now time.Time) float32 {
	if !s.linkDegraded(now) {
		return speed
	}
	limit := float32(s.healthPolicy.SpeedCapMPS)
	return float32(math.Copysign(float64(min(abs32(speed), limit)), float64(speed)))
}

func abs32(v float32) float32 {
	if v < 0 {
		return -v
	}
	return v
}

// resetHealth forgets the board's totals, for a new boot.
func (s *Session) resetHealth() {
	s.health.haveTotals = false
}
