package scenario

import (
	"math"
	"testing"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/signrouter"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/visionsim"
)

// aheadSign sits straight ahead of a robot driving along +x from the origin.
var aheadSign = []signrouter.SignSpec{{X: 1.2, Y: 0, Color: signrouter.SignColorGreen}}

// drivingAt is the chassis at t seconds of driving along +x at 1 m/s.
func drivingAt(t float64) kinematics.AckermannState {
	return kinematics.AckermannState{X: t, Y: 0, Yaw: 0}
}

func TestNewSimCamera_OffIsNil(t *testing.T) {
	t.Parallel()

	if c := newSimCamera(harness.TransportConfig{ScanDelayS: 0.2}, func() float64 { return 0 }, 1); c != nil {
		t.Error("a camera was built with no detection latency or drops")
	}
}

// A late camera reports nothing until the run is older than the delay, then
// reports the sign where it was seen from the old pose but placed through
// the current one: off by exactly the distance driven meanwhile.
func TestSimCamera_DelayPlacesThroughTheCurrentPose(t *testing.T) {
	t.Parallel()

	const delay = 0.3
	now := 0.0
	cam := newSimCamera(harness.TransportConfig{DetectionDelayS: delay}, func() float64 { return now }, 1)
	cfg := visionsim.DefaultConfig()

	var got []signrouter.TrafficSignObservation
	for ; now <= 0.4+1e-9; now += 0.05 {
		obs, ok := cam.detections(aheadSign, drivingAt(now), cfg)
		if now < delay-1e-9 && ok {
			t.Fatalf("detection at %.2f s, before the %.2f s delay had passed", now, delay)
		}
		got = obs
	}
	if len(got) != 1 {
		t.Fatalf("detections at 0.4 s = %d, want 1", len(got))
	}
	// Seen at x=0.1 (1.1 m away), placed from x=0.4: 1.5, 0.3 m past the sign.
	if math.Abs(got[0].WorldXM-1.5) > 1e-6 || math.Abs(got[0].WorldYM) > 1e-6 {
		t.Errorf("detection at (%.4f, %.4f), want (1.5, 0): seen from 0.3 s ago, placed from now",
			got[0].WorldXM, got[0].WorldYM)
	}
}

// Frames drop at the configured rate, one draw per tick however often the
// navigator asks within it.
func TestSimCamera_DropsWholeFramesPerTick(t *testing.T) {
	t.Parallel()

	const rate, ticks = 0.79, 20_000
	now := 0.0
	cam := newSimCamera(harness.TransportConfig{DetectionDropRate: rate}, func() float64 { return now }, 3)
	cfg := visionsim.DefaultConfig()
	st := drivingAt(0)

	dropped := 0
	for i := range ticks {
		now = float64(i) * 0.05
		_, first := cam.detections(aheadSign, st, cfg)
		if _, second := cam.detections(aheadSign, st, cfg); second != first {
			t.Fatalf("tick %d: two asks in one tick disagreed (%v, %v)", i, first, second)
		}
		if !first {
			dropped++
		}
	}
	if got := float64(dropped) / ticks; math.Abs(got-rate) > 0.01 {
		t.Errorf("dropped fraction = %.4f, want %.2f +- 0.01", got, rate)
	}
}
