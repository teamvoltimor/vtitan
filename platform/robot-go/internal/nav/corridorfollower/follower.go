package corridorfollower

import (
	"math"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/navutil"
)

// Params bundles FollowCorridor's optional arguments, which Python passes as
// keyword defaults.
type Params struct {
	// SpeedMPS is the creep speed while the direction is unknown.
	SpeedMPS float64
	// Yaw damps the centering branch against the nearest track axis. Optional
	// because the value is only ever used mod 90 degrees -- the track is a
	// Manhattan world, so this stays as map-free as the rest of the module.
	// Omitted, centering falls back to offset-proportional, which oscillates.
	Yaw *float64
	// ForcedTurnSide overrides which side the back-off and corner branches
	// turn toward.
	//
	// Both otherwise pick the side with more LIDAR clearance, which is right
	// for a plain wall but NOT for a red/green traffic sign: those have a
	// fixed WRO pass-side rule (red outward, green inward) that has nothing
	// to do with which side looks more open. The caller resolves that rule
	// from a world-frame sign observation, which this function cannot do --
	// it only ever sees robot-frame LIDAR.
	ForcedTurnSide TurnSide
	// BelievedWidthM is the corridor currently being creep-followed, from the
	// caller's own running average of width measurements taken this same
	// creep phase. Nil (no readings yet) keeps the plain TurnClearanceM.
	BelievedWidthM *float64
	// RearClearance supplies the rear reading and whether it is measurable at
	// all. See FollowCorridor on why ok=false is not permission to reverse.
	RearClearance func() (clearanceM float64, ok bool)
}

// FollowCorridor creeps along the corridor, centered between whatever walls
// are visible, matching corridor_follower.py's follow_corridor.
//
// Returns a drive command centering the chassis, or a stop if the corridor
// ended before the direction resolved.
func FollowCorridor(
	rangesM []float64,
	anglesRad []float64,
	params Params,
	cfg Config,
) controllers.DriveCommand {
	// Steering is reasoned about here as a physical road-wheel angle and
	// normalized exactly once, on the way out. Holding it in normalized units
	// made every constant a fraction of whatever full lock happened to be, so
	// recalibrating the servo silently retuned the loop.
	maxCenteringRad := cfg.MaxCenteringSteerDeg * math.Pi / navutil.DegreesPerHalfTurn
	centeringGainRadPerM := cfg.CenteringGainDegPerM * math.Pi / navutil.DegreesPerHalfTurn
	maxCorner := navutil.SteeringNormFromAngleRad(
		cfg.MaxCornerSteerDeg*math.Pi/navutil.DegreesPerHalfTurn, cfg.MaxSteeringAngleRad,
	)

	turnClearance := cfg.TurnClearanceM
	if params.BelievedWidthM != nil && *params.BelievedWidthM < cfg.DecisionBoundaryM {
		turnClearance = cfg.NarrowTurnClearanceM
	}

	forward := navutil.ForwardClearance(
		rangesM,
		anglesRad,
		cfg.ForwardArcHalfFovRad,
		cfg.MinValidRangeM,
	)
	left := navutil.NearestRay(rangesM, anglesRad, math.Pi/2)
	right := navutil.NearestRay(rangesM, anglesRad, -math.Pi/2)

	// Something close ahead is a fact about SAFETY, not about the layout, so
	// it is answered first and on the forward minimum: whether or not the
	// corridor has ended, there is no room to drive on. Hoisted out of the
	// corner branch below, which no longer fires on every close wall and so
	// can no longer be relied on to reach this.
	if forward < cfg.MinForwardClearanceM {
		return backOff(params, cfg, left, right, maxCorner, turnClearance)
	}

	if forward < turnClearance && !wayThrough(rangesM, anglesRad, cfg) {
		// The corridor is ending -- close ahead AND nothing open across the
		// wider arc, so this is a wall spanning the track rather than one
		// seen at an angle. Turn toward the side with more room, which is
		// where the track continues, and is the same observation the
		// direction estimator settles on, so the turn and the answer agree.
		//
		// Stopping here instead is a deadlock: with no direction there is no
		// plan to hand over to, so the robot would sit at the corner until
		// the round expired. That was every closed-loop failure of this
		// feature.
		return controllers.DriveCommand{
			SpeedMPS:     params.SpeedMPS * cfg.CornerSpeedScale,
			SteeringNorm: cornerSteer(params.ForcedTurnSide, left, right, maxCorner),
		}
	}

	// Once a side has opened past the end of the inner block it is no longer
	// a corridor wall, and centering against it would steer into the other
	// one. Hold the line instead; the direction estimator is about to settle
	// on the very reading that disqualified it.
	limit := cfg.WideWidthM + cfg.CornerLeakMarginM
	if left > limit || right > limit {
		return controllers.DriveCommand{SpeedMPS: params.SpeedMPS, SteeringNorm: 0.0}
	}

	// More room on the left means the chassis sits right of center, so steer
	// left to correct -- and +1 steering is full left.
	offset := (left - right) / halvesPerWidth
	demandRad := centeringGainRadPerM * offset
	// Damping. Offset alone is 90 degrees out of phase with the control the
	// chassis actually has -- steering sets yaw rate, yaw integrates to
	// heading, heading integrates to position -- so correcting position
	// without regard to heading always overshoots and comes back. Subtracting
	// the heading error takes the corner off that: pointing left of the
	// corridor axis is a reason to steer right even while still left of
	// center. Positive axis offset means the nose is left of the axis, and +1
	// steering is full left, hence minus.
	if params.Yaw != nil {
		demandRad -= cfg.HeadingGain * navutil.AxisOffsetRad(*params.Yaw)
	}
	steerRad := navutil.Clamp(demandRad, -maxCenteringRad, maxCenteringRad)

	return controllers.DriveCommand{
		SpeedMPS:     params.SpeedMPS,
		SteeringNorm: navutil.SteeringNormFromAngleRad(steerRad, cfg.MaxSteeringAngleRad),
	}
}

// backOff handles the too-close-ahead branch.
//
// Reversing swings the nose AWAY from the steer direction, so the command
// inverts the turn sign: that walks the nose toward the open side instead of
// further into the wall it is against.
func backOff(
	params Params,
	cfg Config,
	left, right, maxCorner, turnClearance float64,
) controllers.DriveCommand {
	steering := cornerSteer(params.ForcedTurnSide, left, right, maxCorner)

	// ok=false means the rear sector is unreadable on this mount, which is
	// NOT permission to reverse into it. This was once a single raw ray
	// straight back, which cannot distinguish "open" from "occluded": the
	// occlusion wedges sit either side of that exact bearing and a no-return
	// is substituted with max range, so the gate read 12 m of open road and
	// backed into whatever was behind.
	if params.RearClearance != nil {
		if rear, ok := params.RearClearance(); ok && rear > cfg.MinReverseClearanceM {
			return controllers.DriveCommand{
				SpeedMPS:     -params.SpeedMPS * cfg.ReverseSpeedScale,
				SteeringNorm: -steering,
			}
		}
	}

	// Rear unreadable, which on this mount is ALWAYS -- there is no rear
	// slot, so holding here is not a pause, it is a permanent stop. Nothing
	// can clear it either: the direction estimator never settles because the
	// robot never moves, so the navigator's step is never reached, the stuck
	// detector never runs, and no escape is ever considered.
	//
	// Measured 2026-08-27: a robot started INSIDE the parking bay -- a legal
	// start -- sat at exactly 0.00 m for the whole run, 8/8 scenarios. Front
	// was 0.05-0.19 m against a parking fin while BOTH sides read 12.0 m.
	// There was an open corridor either side and the robot could see it.
	//
	// So pivot toward it instead: creep forward under full lock and let the
	// nose swing out. Gated on the side being genuinely open, so a true dead
	// end -- boxed on three sides with nowhere to pivot to -- still holds,
	// and the safety this branch exists for is preserved.
	if math.Max(left, right) > turnClearance {
		return controllers.DriveCommand{
			SpeedMPS:     params.SpeedMPS * cfg.CornerSpeedScale,
			SteeringNorm: steering,
		}
	}
	return controllers.DriveCommand{SpeedMPS: 0.0, SteeringNorm: steering}
}

// cornerSteer picks the turn direction, honoring a forced side when the
// caller has one.
func cornerSteer(forced TurnSide, left, right, maxCorner float64) float64 {
	turnLeft := left > right
	switch forced {
	case TurnSideLeft:
		turnLeft = true
	case TurnSideRight:
		turnLeft = false
	case TurnSideNone:
	}
	if turnLeft {
		return maxCorner
	}
	return -maxCorner
}

// wayThrough reports whether any bearing in the forward arc is still open
// enough to drive down, matching _way_through.
//
// Dropouts are excluded on the same reasoning, and against the same bound, as
// the direction estimator uses: the gateway substitutes max range (12 m) for
// a no-return, and nothing on a 3 m mat can be further than its diagonal.
// Left in, a single dropped beam would read as wide-open track and veto every
// corner turn on the round.
func wayThrough(rangesM, anglesRad []float64, cfg Config) bool {
	turnArcRad := cfg.TurnArcHalfFovDeg * math.Pi / navutil.DegreesPerHalfTurn

	best := math.Inf(-1)
	found := false
	for i, angle := range anglesRad {
		if i >= len(rangesM) {
			break
		}
		r := rangesM[i]
		if math.Abs(navutil.WrapAngle(angle)) > turnArcRad {
			continue
		}
		if r <= cfg.MinValidRangeM || r >= cfg.MaxInTrackRangeM {
			continue
		}
		if r > best {
			best, found = r, true
		}
	}
	if !found {
		return false
	}
	return best >= cfg.TurnOpenRangeM
}
