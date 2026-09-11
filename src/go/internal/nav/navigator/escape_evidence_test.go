package navigator_test

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/nav/navigator"
)

// wallAheadM is the range the forward rays report in the escape-triggering
// scan: inside the shipped contact distance (0.10 m), which is what
// CollisionAvoidanceController.AssessRisk reads as CRITICAL.
const wallAheadM = 0.08

// openBehindM is the range every other ray reports. The rear must be genuinely
// clear or ComputeEscapeManeuver declines to reverse into it and no escape is
// triggered at all -- an all-sides-blocked scan tests the wrong thing.
const openBehindM = 3.0

// escapeScanRayCount is how many rays the staged scan carries: a full-circle
// sweep at 1-degree resolution.
const escapeScanRayCount = 360

// forwardBlockedHalfWidthRays is how many rays either side of dead ahead are
// pulled in to wallAheadM, matching the narrow forward sector the collision
// controller's own tests block.
const forwardBlockedHalfWidthRays = 4

// criticalScan builds a full-circle scan that is clear everywhere except a
// narrow sector dead ahead, inside contact range: a robot nose-to-nose with a
// wall and free to back away from it.
func criticalScan() controllers.LidarScan {
	ranges := make([]float64, escapeScanRayCount)
	angles := make([]float64, escapeScanRayCount)
	for i := range ranges {
		ranges[i] = openBehindM
		angles[i] = -math.Pi + 2*math.Pi*float64(i)/float64(escapeScanRayCount)
	}
	// Dead ahead (0 rad) is the middle of the sweep.
	center := escapeScanRayCount / 2
	for offset := -forwardBlockedHalfWidthRays; offset <= forwardBlockedHalfWidthRays; offset++ {
		ranges[(center+offset+escapeScanRayCount)%escapeScanRayCount] = wallAheadM
	}
	return controllers.LidarScan{RangesM: ranges, AnglesRad: angles}
}

// TestStep_EscapeTriggerTickKeepsItsEvidence pins the fix for an evidence
// erasure: the trigger tick used to publish the escape and then REBUILD its
// debug snapshot from scratch, discarding the forward clearance, both risk
// levels and the min LIDAR range that had just been measured to decide the
// escape was needed.
//
// That made the one tick that can explain an escape the one tick that does
// not, so bag analysis had to attribute the cause to the PRECEDING tick --
// a different scan, taken before whatever changed.
func TestStep_EscapeTriggerTickKeepsItsEvidence(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	gateway.setPose(0, 0, 0)
	gateway.scan = criticalScan()
	gateway.haveScan = true

	nav.Step()

	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseEscapeTriggered {
		t.Fatalf("Phase = %v, want PhaseEscapeTriggered (the staged scan did not "+
			"trigger an escape, so this test is not exercising the trigger tick)",
			debug.Phase)
	}

	// The maneuver fields describe what the escape DID; these describe why.
	// Both have to survive the same tick for the record to be usable.
	if debug.ForwardClearanceM == nil {
		t.Error("ForwardClearanceM is nil on the trigger tick: the clearance that " +
			"justified the escape was erased")
	}
	if debug.Risk == nil {
		t.Error("Risk is nil on the trigger tick")
	}
	if debug.EscapeRisk == nil {
		t.Error("EscapeRisk is nil on the trigger tick: the escape's own risk read " +
			"is the direct trigger condition")
	}
	if debug.MinLidarRangeM == nil {
		t.Error("MinLidarRangeM is nil on the trigger tick")
	}

	// ...and the maneuver fields must still be there, i.e. the fix kept the
	// evidence without dropping what it replaced.
	if debug.ActiveManeuverType == nil {
		t.Error("ActiveManeuverType is nil: the maneuver description was lost")
	}
	if debug.ManeuverFramesLeft == nil {
		t.Error("ManeuverFramesLeft is nil")
	}
	if debug.EscapeCount == nil {
		t.Error("EscapeCount is nil")
	}
	if debug.CommandedSpeedMPS == nil || debug.CommandedSteerNorm == nil {
		t.Error("the commanded speed/steering the escape published were not recorded")
	}
}

// TestStep_ActiveManeuverContinuationRebuildsSnapshot is the other half of
// the same behavior: a CONTINUATION tick has no earlier evidence to keep
// (Step returns before any clearance is computed), so it must still produce
// a populated snapshot from the pose alone rather than carrying the trigger
// tick's stale clearance forward as if it had been re-measured.
func TestStep_ActiveManeuverContinuationRebuildsSnapshot(t *testing.T) {
	t.Parallel()

	nav, gateway := newNavigator(t)
	gateway.setPose(0, 0, 0)
	gateway.scan = criticalScan()
	gateway.haveScan = true

	nav.Step() // trigger
	if nav.DebugSnapshot().Phase != navigator.PhaseEscapeTriggered {
		t.Fatalf("first Step: Phase = %v, want PhaseEscapeTriggered", nav.DebugSnapshot().Phase)
	}

	nav.Step() // continuation

	debug := nav.DebugSnapshot()
	if debug.Phase != navigator.PhaseActiveManeuver {
		t.Fatalf("second Step: Phase = %v, want PhaseActiveManeuver", debug.Phase)
	}
	if debug.ForwardClearanceM != nil {
		t.Error("ForwardClearanceM is set on a continuation tick: no clearance was " +
			"measured this tick, so reporting one would be a stale reading " +
			"presented as a fresh measurement")
	}
	if debug.ActiveManeuverType == nil {
		t.Error("ActiveManeuverType is nil on a continuation tick")
	}
	if debug.PoseX == nil || debug.PoseY == nil || debug.PoseYaw == nil {
		t.Error("the continuation tick recorded no pose")
	}
}
