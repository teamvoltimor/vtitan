package scenario

import (
	"slices"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/parking"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/collision"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/corpus"
)

// scoreInput groups score's inputs, replacing a fourteen-argument signature
// whose adjacent float64s and ints were easy to transpose silently.
type scoreInput struct {
	sc            corpus.Scenario
	gw            simGateway
	nav           *navigator.Navigator
	steps         int
	distanceM     float64
	maxSpeedMPS   float64
	minRangeM     float64
	contactCount  int
	targetLaps    int
	surface       collision.ContactSurface
	stuck         bool
	passSideWrong []int
	trueSigns     int
	lapSteps      []int
	simTimeS      float64
}

// score turns a finished run into its Result.
func (r *NativeRunner) score(in scoreInput) Result {
	// collided is derived from the surface rather than passed alongside it,
	// so the two can never disagree about whether the run ended in contact.
	collided := in.surface != collision.SurfaceNone
	laps := in.nav.LapsCompleted()
	st := in.gw.State()
	cx, cy := in.gw.CollisionXY()

	// PassSideViolationSigns/PassSideViolation are only ever populated by an
	// Obstacles Challenge run (nav.SignRouter() nil for Open), matching
	// SimResult's own fields -- an empty sign-router-less run reports zero
	// violations, not "unknown."
	// The VERDICT is passSideWrong, scored by passSideScorer from the true
	// layout against the true pose, and it is what ended the run. The
	// router's own set is reported alongside as a measure of DISCOVERY
	// quality only -- it is computed in the believed frame and is cleared
	// every lap, so it can neither end a round nor be counted as one.
	var routerWrongSide []int
	var passRecords []signrouter.PassRecord
	var discoveredSigns int
	if sr := in.nav.SignRouter(); sr != nil {
		for index := range sr.WrongSideViolations() {
			routerWrongSide = append(routerWrongSide, index)
		}
		slices.Sort(routerWrongSide)
		passRecords = sr.PassRecords()
		discoveredSigns = len(sr.Signs())
	}
	passSideViolation := len(in.passSideWrong) > 0

	success := !collided && !in.stuck && !passSideViolation && !resTimedOut(in.steps, r.maxSteps, laps, in.targetLaps)

	// Parked is nil for a scenario with no parking lot, matching
	// SimResult.parked's None. ParkPoints additionally scores the final
	// pose against the WRO 15/7/0 tiers (parking.ScorePark) -- an addition
	// beyond Python's plain boolean, since that scorer was ported standalone
	// and never wired into SimResult either.
	var parked *bool
	var parkPoints *int
	if pc := in.nav.ParkController(); pc != nil {
		p := pc.IsDone() && !pc.IsTimedOut()
		parked = &p
		score := parking.ScorePark(st.X, st.Y, st.Yaw, pc.Zone(), r.parkCfg)
		points := score.Points
		parkPoints = &points
	}

	return Result{
		TerminalSurface:        in.surface.String(),
		Scenario:               in.sc.ID,
		PassSideViolationSigns: in.passSideWrong,
		RouterWrongSideSigns:   routerWrongSide,
		DiscoveredSigns:        discoveredSigns,
		TrueSigns:              in.trueSigns,
		PassRecords:            passRecords,
		LapStepIndices:         in.lapSteps,
		CollisionXY:            []float64{cx, cy},
		FinalPose:              []float64{st.X, st.Y, st.Yaw},
		Parked:                 parked,
		ParkPoints:             parkPoints,
		SimTimeS:               in.simTimeS,
		DistanceM:              in.distanceM,
		MaxSpeedMPS:            in.maxSpeedMPS,
		AvgSpeedMPS:            avgSpeed(in.distanceM, in.simTimeS),
		MinLidarRangeM:         orZero(in.minRangeM),
		TargetLaps:             in.targetLaps,
		LapsCompleted:          laps,
		Steps:                  in.steps,
		ContactCount:           in.contactCount,
		Collided:               collided,
		PassSideViolation:      passSideViolation,
		TimedOut:               resTimedOut(in.steps, r.maxSteps, laps, in.targetLaps),
		Stuck:                  in.stuck,
		Success:                success,
		OverTime:               in.simTimeS > r.roundTimeLimitS,
	}
}
