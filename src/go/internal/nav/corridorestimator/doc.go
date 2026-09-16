// Package corridorestimator estimates corridor widths from LIDAR alone,
// without being told the layout. Ports src/navigation/corridor_estimator.py.
//
// Every other part of navigation is handed the corridor widths from scenario
// metadata: the planned path is built from them and the localizer matches
// scans against a wall model built from them. That is only legitimate if
// someone measured the mat and wrote the file first -- and WRO places the
// inner walls randomly before each round, so the true widths cannot be known
// in advance. This closes that gap.
//
// It exploits the one thing the rules DO guarantee: each corridor is either
// 0.6 m or 1.0 m. The robot never has to measure a width, only decide between
// two values far enough apart to be a call rather than a measurement, given
// the LIDAR's own sigma. See
// adr:0057-blind-corridor-follower-and-width.
//
// # Why NARROW is the safe default, and where it is not
//
// An unknown corridor reports NARROW rather than nothing, because planning a
// 1.0 m corridor as if it were 0.6 m puts the path nearer the OUTER wall,
// which stays inside the true corridor. The converse does not: planning a
// 0.6 m corridor as if it were 1.0 m puts the path 0.15 m from the inner
// block face, closer than the chassis half-diagonal (0.180 m), so the corner
// clips it mid-turn.
//
// The Obstacles Challenge is not that round. Its corridors are all 1.0 m by
// RULE, so assuming narrow there is not conservative, it is known to be wrong
// for every corridor -- and it cost real runs, because the robot met a sign
// before MinSamples readings had accumulated to correct the default.
// WithFixedWidth exists for exactly that case. See
// adr:0057-blind-corridor-follower-and-width.
//
// # Why attribution is by heading, not position
//
// Attributing a width measurement via the position estimate is circular: the
// position comes from matching against a wall model built from the widths
// being estimated, so a wrong belief mis-attributes the very reading that
// would have corrected it, and the error locks in. Heading breaks that loop --
// it comes from the IMU and owes nothing to the map.
package corridorestimator
