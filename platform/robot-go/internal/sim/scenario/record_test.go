package scenario

import (
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/foxglove/mcap/go/mcap"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/nav/controllers"
	navv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/nav/v1"
	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
)

// TestNewSimRecorder_NilWhenOff pins the "off" contract the run loop relies
// on: an empty root yields a nil recorder whose methods are all no-ops, so
// the hot loop needs no branch and an unrecorded sweep pays nothing.
func TestNewSimRecorder_NilWhenOff(t *testing.T) {
	t.Parallel()

	rec, err := newSimRecorder("", "open_0000")
	if err != nil {
		t.Fatalf("newSimRecorder(\"\"): %v", err)
	}
	if rec != nil {
		t.Fatalf("newSimRecorder(\"\") = %v, want nil", rec)
	}
	if err := rec.close(); err != nil {
		t.Errorf("close on a nil recorder: %v", err)
	}
	if err := rec.tick(controllers.LidarScan{}, false, nil, 0.05); err != nil {
		t.Errorf("tick on a nil recorder: %v", err)
	}
}

// TestScanToProto_RebuildsTheAngleFan checks the conversion the bag depends
// on. The simulator carries an explicit angle per sample; LaserScan (and
// this repo's mirror of it) carries a start angle and a fixed increment, so
// the fan has to be re-expressed rather than copied. A wrong increment puts
// every ray at the wrong bearing in Foxglove while the ranges still look
// plausible -- the kind of error that reads as a nav bug.
func TestScanToProto_RebuildsTheAngleFan(t *testing.T) {
	t.Parallel()

	scan := controllers.LidarScan{
		RangesM:   []float64{1.0, 2.0, 0.5, 3.0},
		AnglesRad: []float64{-math.Pi, -math.Pi / 2, 0, math.Pi / 2},
	}
	got := scanToProto(scan, 0)

	if len(got.GetRanges()) != len(scan.RangesM) {
		t.Fatalf("ranges = %d, want %d", len(got.GetRanges()), len(scan.RangesM))
	}
	if math.Abs(float64(got.GetAngleMin())-(-math.Pi)) > 1e-6 {
		t.Errorf("AngleMin = %v, want -pi", got.GetAngleMin())
	}
	if math.Abs(float64(got.GetAngleMax())-math.Pi/2) > 1e-6 {
		t.Errorf("AngleMax = %v, want pi/2", got.GetAngleMax())
	}
	// Four samples spanning -pi..pi/2 is three steps of pi/2.
	if math.Abs(float64(got.GetAngleIncrement())-math.Pi/2) > 1e-6 {
		t.Errorf("AngleIncrement = %v, want pi/2", got.GetAngleIncrement())
	}
	// range_min/max describe the SWEEP's extent, which is what a viewer uses
	// to scale its colour ramp.
	if math.Abs(float64(got.GetRangeMin())-0.5) > 1e-6 {
		t.Errorf("RangeMin = %v, want 0.5", got.GetRangeMin())
	}
	if math.Abs(float64(got.GetRangeMax())-3.0) > 1e-6 {
		t.Errorf("RangeMax = %v, want 3.0", got.GetRangeMax())
	}
	if got.GetFrameId() != scanFrameID {
		t.Errorf("FrameId = %q, want %q", got.GetFrameId(), scanFrameID)
	}
}

// TestScanToProto_EmptyScanHasNoInfiniteRange guards the degenerate case:
// range_min starts at +Inf while scanning for the minimum, and an empty
// sweep would ship that Inf into the bag, where a viewer's colour ramp
// collapses.
func TestScanToProto_EmptyScanHasNoInfiniteRange(t *testing.T) {
	t.Parallel()

	got := scanToProto(controllers.LidarScan{}, 0)
	if math.IsInf(float64(got.GetRangeMin()), 0) || math.IsInf(float64(got.GetRangeMax()), 0) {
		t.Errorf("empty scan produced RangeMin=%v RangeMax=%v, want finite",
			got.GetRangeMin(), got.GetRangeMax())
	}
}

// TestSimRunsRootFor_PrefersTheExplicitDir checks the override, so a caller
// (or a test) can keep bags out of the repo's data tree entirely.
func TestSimRunsRootFor_PrefersTheExplicitDir(t *testing.T) {
	t.Parallel()

	want := filepath.Join("some", "where")
	got, err := SimRunsRootFor(want, time.Now())
	if err != nil {
		t.Fatalf("SimRunsRootFor: %v", err)
	}
	if got != want {
		t.Errorf("SimRunsRootFor(%q) = %q, want it unchanged", want, got)
	}
}

// TestSimRecorder_WritesBothSubjectsOnASimClock is the end-to-end shape
// check: a bag Foxglove can open, carrying both channels as protobuf, on the
// SIMULATION clock.
//
// The clock matters more than it looks. A sweep runs concurrently and
// finishes in seconds of wall time, so wall-clock stamps would compress a
// 110-second round into a smear and make the timeline useless for scrubbing.
func TestSimRecorder_WritesBothSubjectsOnASimClock(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	rec, err := newSimRecorder(dir, "open_0042")
	if err != nil {
		t.Fatalf("newSimRecorder: %v", err)
	}

	const dt = 0.05
	const ticks = 4
	scan := controllers.LidarScan{RangesM: []float64{1, 2}, AnglesRad: []float64{-1, 1}}
	for range ticks {
		// A nil navigator would panic in tick, so drive the recorder's writes
		// directly here; the navigator's own ToProto is covered in its package.
		logTime := rec.simClockNanos
		if err := rec.run.WriteMessage(sensorv1.ScanSubject, scanToProto(scan, logTime), logTime); err != nil {
			t.Fatalf("writing scan: %v", err)
		}
		if err := rec.run.WriteMessage(navv1.NavigatorDebugSubject, &navv1.NavigatorDebug{}, logTime); err != nil {
			t.Fatalf("writing nav debug: %v", err)
		}
		rec.simClockNanos += uint64(dt * nanosPerSecond)
	}
	if err := rec.close(); err != nil {
		t.Fatalf("close: %v", err)
	}

	// The run directory and bag are named for the SCENARIO, not a timestamp:
	// a sweep starts hundreds of runs inside one second.
	bag := filepath.Join(dir, "open_0042", "open_0042_0.mcap")
	f, err := os.Open(bag) //nolint:gosec // path is composed from t.TempDir
	if err != nil {
		t.Fatalf("opening the bag: %v", err)
	}
	defer f.Close()

	reader, err := mcap.NewReader(f)
	if err != nil {
		t.Fatalf("reading the bag: %v", err)
	}
	info, err := reader.Info()
	if err != nil {
		t.Fatalf("bag info: %v", err)
	}

	if got := info.Statistics.MessageCount; got != ticks*2 {
		t.Errorf("bag holds %d messages, want %d", got, ticks*2)
	}
	topics := map[string]string{}
	for _, ch := range info.Channels {
		topics[ch.Topic] = ch.MessageEncoding
	}
	for _, subject := range []string{sensorv1.ScanSubject, navv1.NavigatorDebugSubject} {
		encoding, ok := topics[subject]
		if !ok {
			t.Errorf("bag has no channel for %q", subject)
			continue
		}
		if encoding != "protobuf" {
			t.Errorf("channel %q encoding = %q, want protobuf", subject, encoding)
		}
	}
	// Sim time, not wall time: four ticks at 50 ms span 150 ms end to end.
	spanNanos := info.Statistics.MessageEndTime - info.Statistics.MessageStartTime
	if want := uint64((ticks - 1) * dt * nanosPerSecond); spanNanos != want {
		t.Errorf("bag spans %d ns, want %d (the simulation clock, not the wall clock)",
			spanNanos, want)
	}
}
