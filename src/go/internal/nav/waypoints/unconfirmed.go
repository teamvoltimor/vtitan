package waypoints

import "github.com/teamvoltimor/vtitan/src/go/internal/nav/trackmodel"

// UnconfirmedSections is the set of corridors whose width the PLAN still
// treats as assumed rather than measured, matching calculate_waypoints'
// `unconfirmed_sections` argument.
//
// A bit per section rather than a set type: there are exactly four, they are
// fixed at compile time, and a fixed-size value can be passed by value and
// compared with == without any allocation on a path the planner runs on
// every replan.
//
// The zero value means EVERY section is confirmed, which is the sighted case
// (a told width is confirmed by definition) and the behaviour every caller
// had before the belief was modelled at all.
type UnconfirmedSections struct {
	North bool
	South bool
	East  bool
	West  bool
}

// AllConfirmed is the zero value, named so a caller that genuinely has no
// belief to express says so rather than passing a bare literal.
func AllConfirmed() UnconfirmedSections { return UnconfirmedSections{} }

// AllUnconfirmed is the state a blind round starts in: every corridor is on
// the narrow prior and none has been measured.
func AllUnconfirmed() UnconfirmedSections {
	return UnconfirmedSections{North: true, South: true, East: true, West: true}
}

// Confirmed reports whether section's width has been measured. It is the
// complement of the set's membership, and is what CenterBiasForCorridor
// takes, so callers never have to negate at the call site.
func (u UnconfirmedSections) Confirmed(section trackmodel.Section) bool {
	return !u.Contains(section)
}

// Contains reports whether section is still unconfirmed.
func (u UnconfirmedSections) Contains(section trackmodel.Section) bool {
	switch section {
	case trackmodel.North:
		return u.North
	case trackmodel.South:
		return u.South
	case trackmodel.East:
		return u.East
	case trackmodel.West:
		return u.West
	default:
		// An unknown section cannot have been measured, so treating it as
		// unconfirmed is the conservative answer: it keeps the inner bias,
		// which is the side with margin to spare.
		return true
	}
}

// Set marks section as unconfirmed (true) or confirmed (false).
func (u *UnconfirmedSections) Set(section trackmodel.Section, unconfirmed bool) {
	switch section {
	case trackmodel.North:
		u.North = unconfirmed
	case trackmodel.South:
		u.South = unconfirmed
	case trackmodel.East:
		u.East = unconfirmed
	case trackmodel.West:
		u.West = unconfirmed
	}
}
