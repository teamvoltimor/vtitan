package directionestimator

import "github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"

// Estimator is a running direction estimate, settled by agreeing
// observations -- the Go port of Python's DirectionEstimator class.
//
// Votes rather than trusting a single scan: a ray slipping past a block
// corner produces brief, clustered misreadings, and one of those arriving
// first should not decide the round. Not safe for concurrent use -- callers
// own one Estimator per navigation run, same as the Python original.
type Estimator struct {
	minVotes int
	votes    map[Direction]int
	settled  *Direction
}

// NewEstimator builds an Estimator requiring minVotes agreeing observations
// before settling (see Config.MinVotes for the Python tuning default).
func NewEstimator(minVotes int) *Estimator {
	return &Estimator{
		minVotes: minVotes,
		votes:    map[Direction]int{Clockwise: 0, Counterclockwise: 0},
	}
}

// Direction returns the settled direction and true, or ok=false until
// enough evidence agrees.
func (e *Estimator) Direction() (dir Direction, ok bool) {
	if e.settled == nil {
		return 0, false
	}
	return *e.settled, true
}

// IsSettled reports whether the direction has been decided.
func (e *Estimator) IsSettled() bool {
	return e.settled != nil
}

// Votes returns the current vote tally per direction, for telemetry -- a
// copy, so callers cannot perturb the real count through it.
func (e *Estimator) Votes() map[Direction]int {
	return map[Direction]int{
		Clockwise:        e.votes[Clockwise],
		Counterclockwise: e.votes[Counterclockwise],
	}
}

// Settle adopts dir outright, without accumulating votes -- for evidence
// that is conclusive on its own rather than statistical (currently only
// DirectionFromParkingBay, where the track's design fixes the answer and
// no number of further scans could improve on it). Deliberately not a
// general escape hatch: settling wrongly is worse than settling late, so a
// second call is ignored, meaning a bootstrap can never overwrite a
// direction already committed.
func (e *Estimator) Settle(dir Direction) {
	if e.settled == nil {
		e.settled = &dir
	}
}

// Observe folds one scan in and reports whether this observation settled
// the direction.
func (e *Estimator) Observe(scan controllers.LidarScan, yaw float64, cfg Config) bool {
	if e.settled != nil {
		return false
	}
	inferred, ok := InferDirection(scan, yaw, cfg)
	if !ok {
		return false
	}

	e.votes[inferred]++
	if e.votes[inferred] < e.minVotes {
		return false
	}
	e.settled = &inferred
	return true
}
