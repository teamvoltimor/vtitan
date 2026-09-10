// Package corridorfollower drives along a corridor without a map, a plan, or
// a travel direction. Ports src/navigation/corridor_follower.py.
//
// There is a gap at the start of a blind round: the robot cannot follow a
// planned path until it knows which way round the loop it is going, and it
// cannot learn that without driving. Something has to move it in between.
//
// Following the corridor needs none of the missing information. Both walls
// are visible, so steering to sit between them is purely reactive -- no
// position, no layout, no direction. This is deliberately NOT a fallback for
// a lost robot: it holds the corridor for the meter or so before the
// direction resolves, then hands over to the real path.
//
// Starting on the assumed direction instead does not work. The path for the
// wrong direction runs the opposite way down the same corridor, so its
// lookahead point is BEHIND the robot and pure pursuit turns it around inside
// the corridor.
//
// # Do not re-add a dead zone on the turn comparison
//
// Tried and reverted 2026-08-02: a dead zone on the `left > right` test,
// holding straight instead of committing to a side when the two were within a
// few centimeters, meant to filter the occasional noisy scan. Even sized to
// real LIDAR noise (~3 cm) it regressed multiple blind Open Challenge fixtures
// into the 180 s round limit or left them oscillating near a corner: in a
// narrow (0.6 m) corridor the asymmetry signal grows slowly approaching a
// turn, so any dead zone eats the same margin the back-off branch depends on,
// disproportionately to the noise it filtered.
//
// The wrong-side steer it was meant to fix is now largely absorbed by the
// navigator's heading-aware ReplacePath reseek instead. Do not re-attempt one
// here without re-measuring against the full Open Challenge sim battery, not
// just the fixture that motivates it.
package corridorfollower
