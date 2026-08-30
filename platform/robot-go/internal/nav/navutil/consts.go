package navutil

import "math"

// DegreesPerHalfTurn is the number of degrees in a half turn (180deg), used to
// convert degree-unit tuning defaults to radians. It is the single source of
// truth for the degToRadTurn / degreesPerHalfTurn constants previously
// duplicated across the nav and config packages.
const DegreesPerHalfTurn = 180.0

// Half divides evenly by two -- named rather than a bare "/ 2.0" per this
// repo's no-magic-numbers convention. It is the single source of truth for the
// halfOf constant previously duplicated across several packages.
const Half = 2.0

// QuarterTurnRad is a quarter turn (90deg) in radians -- the WRO track is a
// Manhattan world, so "aligned with a corridor" means "within tolerance of a
// multiple of 90deg" regardless of which corridor. It is the single source of
// truth for the quarterTurn / quarterTurnRad constants previously duplicated.
const QuarterTurnRad = math.Pi / 2

// SteeringNormLimit is the saturation bound of a normalised steering command
// in [-1, 1]. It is the single source of truth for the normLimit /
// steerNormLimit constants previously duplicated.
const SteeringNormLimit = 1.0
