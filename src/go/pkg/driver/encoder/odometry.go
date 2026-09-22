package encoder

import "github.com/teamvoltimor/vtitan/src/go/pkg/portable/quadrature"

// Odometry is one full sample from the encoder; an alias so encoder's
// callers keep their spelling after the type moved to pkg/portable.
type Odometry = quadrature.Odometry
