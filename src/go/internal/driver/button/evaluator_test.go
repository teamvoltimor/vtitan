package button_test

import (
	"math"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/button"
)

// rawSample is one scripted raw (undebounced) pin reading fed to the
// Evaluator at a given offset from t0 -- this is the "fake pin-state
// source" the button package's tests are built on, standing in for a real
// GPIO line without any hardware or gpiocdev dependency.
type rawSample struct {
	AtMS    int64
	Pressed bool
}

// wantEvent is one Event expected out of the sample sequence above, in
// order, with HeldSec checked to within eventEpsilonSec.
type wantEvent struct {
	Kind    button.Kind
	HeldSec float64
}

// eventEpsilonSec is the tolerance used when comparing a produced Event's
// HeldSec against the expected value in the tables below -- both sides are
// computed from the same millisecond offsets, so any real mismatch is far
// larger than floating-point noise.
const eventEpsilonSec = 0.0005

// testThresholds is a fixed, readable-in-milliseconds Thresholds used by
// every test below: a short debounce window and small hold thresholds keep
// the scripted sample sequences short without losing generality --
// evaluator.go's logic is unitless w.r.t. actual duration magnitudes.
var testThresholds = button.Thresholds{
	DebounceInterval:       20 * time.Millisecond,
	LongPressThreshold:     100 * time.Millisecond,
	ShutdownPressThreshold: 300 * time.Millisecond,
}

func TestEvaluator_Sample(t *testing.T) {
	t.Parallel()

	tests := []struct {
		Name       string
		Thresholds button.Thresholds
		Samples    []rawSample
		Want       []wantEvent
	}{
		{
			Name:       "raw noise inside the debounce window never emits a press",
			Thresholds: testThresholds,
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 5, Pressed: false},
				{AtMS: 10, Pressed: true},
				{AtMS: 15, Pressed: false},
			},
			Want: nil,
		},
		{
			Name:       "a raw press held past the debounce window emits Pressed",
			Thresholds: testThresholds,
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 25, Pressed: true},
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
			},
		},
		{
			Name:       "a quick press-release below both thresholds emits ShortPress on release",
			Thresholds: testThresholds,
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 25, Pressed: true}, // debounced press at 25ms (HeldSec 0 at this instant)
				{AtMS: 50, Pressed: false},
				{AtMS: 75, Pressed: false}, // debounced release at 75ms, held = 75-25 = 50ms
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindShortPress, HeldSec: 0.050},
			},
		},
		{
			Name: "a hold past LongPressThreshold fires exactly once, not on every later sample",
			Thresholds: button.Thresholds{
				DebounceInterval:       10 * time.Millisecond,
				LongPressThreshold:     50 * time.Millisecond,
				ShutdownPressThreshold: 500 * time.Millisecond,
			},
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 15, Pressed: true}, // debounced press at 15ms
				{AtMS: 60, Pressed: true}, // held 45ms, still under 50ms threshold
				{AtMS: 70, Pressed: true}, // held 55ms, crosses 50ms threshold -> LongPress
				{AtMS: 80, Pressed: true}, // held 65ms, still past threshold -- must NOT fire again
				{AtMS: 90, Pressed: true}, // held 75ms, same
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindLongPress, HeldSec: 0.055},
			},
		},
		{
			Name: "a hold past both thresholds fires LongPress then ShutdownPress, each once",
			Thresholds: button.Thresholds{
				DebounceInterval:       10 * time.Millisecond,
				LongPressThreshold:     50 * time.Millisecond,
				ShutdownPressThreshold: 150 * time.Millisecond,
			},
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 15, Pressed: true},  // debounced press at 15ms
				{AtMS: 70, Pressed: true},  // held 55ms -> LongPress
				{AtMS: 100, Pressed: true}, // held 85ms, under shutdown threshold -- no event
				{AtMS: 170, Pressed: true}, // held 155ms -> ShutdownPress
				{AtMS: 200, Pressed: true}, // held 185ms, past both -- no further events
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindLongPress, HeldSec: 0.055},
				{Kind: button.KindShutdownPress, HeldSec: 0.155},
			},
		},
		{
			Name: "release after a hold threshold already fired emits Released, not ShortPress",
			Thresholds: button.Thresholds{
				DebounceInterval:       10 * time.Millisecond,
				LongPressThreshold:     50 * time.Millisecond,
				ShutdownPressThreshold: 500 * time.Millisecond,
			},
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 15, Pressed: true}, // debounced press at 15ms
				{AtMS: 70, Pressed: true}, // held 55ms -> LongPress
				{AtMS: 100, Pressed: false},
				{AtMS: 115, Pressed: false}, // debounced release at 115ms
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindLongPress, HeldSec: 0.055},
				{Kind: button.KindReleased, HeldSec: 0.100},
			},
		},
		{
			Name: "a second press after release re-arms the hold thresholds",
			Thresholds: button.Thresholds{
				DebounceInterval:       10 * time.Millisecond,
				LongPressThreshold:     50 * time.Millisecond,
				ShutdownPressThreshold: 500 * time.Millisecond,
			},
			Samples: []rawSample{
				{AtMS: 0, Pressed: true},
				{AtMS: 15, Pressed: true}, // press #1 debounced at 15ms
				{AtMS: 70, Pressed: true}, // held 55ms -> LongPress #1
				{AtMS: 100, Pressed: false},
				{
					AtMS:    115,
					Pressed: false,
				}, // release #1 debounced at 115ms, held 100ms -> Released

				{AtMS: 200, Pressed: true},
				{AtMS: 215, Pressed: true}, // press #2 debounced at 215ms
				{
					AtMS:    270,
					Pressed: true,
				}, // held 55ms since press #2 -> LongPress #2, proving the flag reset
			},
			Want: []wantEvent{
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindLongPress, HeldSec: 0.055},
				{Kind: button.KindReleased, HeldSec: 0.100},
				{Kind: button.KindPressed, HeldSec: 0},
				{Kind: button.KindLongPress, HeldSec: 0.055},
			},
		},
	}

	for _, tt := range tests {
		t.Run(tt.Name, func(t *testing.T) {
			t.Parallel()

			eval := button.NewEvaluator(tt.Thresholds)
			t0 := time.Date(2026, 1, 1, 0, 0, 0, 0, time.UTC)

			var got []wantEvent
			for _, s := range tt.Samples {
				now := t0.Add(time.Duration(s.AtMS) * time.Millisecond)
				if ev := eval.Sample(s.Pressed, now); ev != nil {
					got = append(got, wantEvent{Kind: ev.Kind, HeldSec: ev.HeldSec})
				}
			}

			assertEvents(t, got, tt.Want)
		})
	}
}

func assertEvents(t *testing.T, got, want []wantEvent) {
	t.Helper()

	if len(got) != len(want) {
		t.Fatalf("got %d events %v, want %d events %v", len(got), got, len(want), want)
	}
	for i := range want {
		if got[i].Kind != want[i].Kind {
			t.Errorf("event[%d].Kind = %v, want %v", i, got[i].Kind, want[i].Kind)
		}
		if math.Abs(got[i].HeldSec-want[i].HeldSec) > eventEpsilonSec {
			t.Errorf(
				"event[%d].HeldSec = %v, want %v (+/- %v)",
				i,
				got[i].HeldSec,
				want[i].HeldSec,
				eventEpsilonSec,
			)
		}
	}
}
