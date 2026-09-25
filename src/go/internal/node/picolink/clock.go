package picolink

import "time"

// ClockSync is the latest clock estimate from a Ping/Pong round trip.
//
// Host times are microseconds on the monotonic clock since Epoch, so a wall
// clock step (NTP) cannot corrupt the estimate; HostTime converts back to a
// time.Time. The residual error of OffsetUS is bounded by RTT/2 (the Pong
// may have been stamped anywhere inside the round trip), which is the
// number go-future.md section 4.6 asks to state rather than assume zero.
type ClockSync struct {
	// Epoch is the host instant host microsecond 0 refers to.
	Epoch time.Time
	// RTT is the round trip of the Ping this estimate came from.
	RTT time.Duration
	// OffsetUS is board clock minus host clock, in microseconds:
	// boardTime - (hostSend+hostRecv)/2.
	OffsetUS int64
	// At is when the Pong arrived.
	At time.Time
}

// HostTime converts a board timestamp into host time with this estimate.
func (c ClockSync) HostTime(boardUS uint64) time.Time {
	return c.Epoch.Add(time.Duration(int64(boardUS)-c.OffsetUS) * time.Microsecond)
}

// clockFrom is the SNTP-style estimate from one round trip: the board
// stamped its clock at some point between hostSend and hostRecv, taken as
// the midpoint.
func clockFrom(hostSendUS, hostRecvUS, boardUS uint64) (rtt time.Duration, offsetUS int64) {
	rttUS := hostRecvUS - hostSendUS
	mid := hostSendUS + rttUS/2
	return time.Duration(rttUS) * time.Microsecond, int64(boardUS) - int64(mid)
}
