package scenario

import (
	"math"
	"os"
	"path/filepath"
	"testing"
	"time"

	"github.com/foxglove/mcap/go/mcap"

	"github.com/teamvoltimor/vtitan/src/go/internal/nav/controllers"
	"github.com/teamvoltimor/vtitan/src/go/internal/recording"
	"github.com/teamvoltimor/vtitan/src/go/internal/sim/kinematics"
	"github.com/teamvoltimor/vtitan/src/go/test/bagreplay"
)

// TestNewSimRecorder_NilWhenOff pins the "off" contract the run loop relies
// on: an empty root yields a nil recorder whose methods are all no-ops, so
// the hot loop needs no branch and an unrecorded sweep pays nothing.
func TestNewSimRecorder_NilWhenOff(t *testing.T) {
	t.Parallel()

	rec, err := newSimRecorder("", "open_0000", recorderGeometry{WheelRadiusM: 0.035, MaxSteerRad: 1.0})
	if err != nil {
		t.Fatalf("newSimRecorder(\"\"): %v", err)
	}
	if rec != nil {
		t.Fatalf("newSimRecorder(\"\") = %v, want nil", rec)
	}
	if closeErr := rec.close(); closeErr != nil {
		t.Errorf("close on a nil recorder: %v", closeErr)
	}
	if tickErr := rec.tick(controllers.LidarScan{}, false, nil, kinematics.AckermannState{}, 0, 0.05); tickErr != nil {
		t.Errorf("tick on a nil recorder: %v", tickErr)
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
	got := scanToCDR(scan, 0)

	if len(got.Ranges) != len(scan.RangesM) {
		t.Fatalf("ranges = %d, want %d", len(got.Ranges), len(scan.RangesM))
	}
	if math.Abs(float64(got.AngleMin)-(-math.Pi)) > 1e-6 {
		t.Errorf("AngleMin = %v, want -pi", got.AngleMin)
	}
	if math.Abs(float64(got.AngleMax)-math.Pi/2) > 1e-6 {
		t.Errorf("AngleMax = %v, want pi/2", got.AngleMax)
	}
	// Four samples spanning -pi..pi/2 is three steps of pi/2.
	if math.Abs(float64(got.AngleIncr)-math.Pi/2) > 1e-6 {
		t.Errorf("AngleIncrement = %v, want pi/2", got.AngleIncr)
	}
	// range_min/max describe the SWEEP's extent, which is what a viewer uses
	// to scale its colour ramp.
	if math.Abs(float64(got.RangeMin)-0.5) > 1e-6 {
		t.Errorf("RangeMin = %v, want 0.5", got.RangeMin)
	}
	if math.Abs(float64(got.RangeMax)-3.0) > 1e-6 {
		t.Errorf("RangeMax = %v, want 3.0", got.RangeMax)
	}
	if got.FrameID != scanFrameID {
		t.Errorf("FrameId = %q, want %q", got.FrameID, scanFrameID)
	}
}

// TestEncodeLaserScan_RoundTripsThroughTheRepoDecoder is the check that
// matters most: the CDR this writes is decoded by test/bagreplay's own
// reader, the one that reads REAL rosbag2 recordings off the robot. If the
// two agree, the bytes are genuine ROS2 CDR rather than a plausible-looking
// buffer -- and Foxglove, which uses the same layout rules, will render it.
func TestEncodeLaserScan_RoundTripsThroughTheRepoDecoder(t *testing.T) {
	t.Parallel()

	want := recording.LaserScanCDR{
		FrameID:      "lidar_link",
		StampSec:     12,
		StampNanosec: 340000000,
		AngleMin:     -math.Pi,
		AngleMax:     math.Pi,
		AngleIncr:    0.0175,
		RangeMin:     0.15,
		RangeMax:     8.0,
		Ranges:       []float32{1.5, 2.25, 0.75, 8.0},
	}
	got, err := bagreplay.DecodeLaserScan(recording.EncodeLaserScan(want))
	if err != nil {
		t.Fatalf("DecodeLaserScan on our own encoding: %v", err)
	}
	if got.AngleMin != want.AngleMin || got.AngleMax != want.AngleMax ||
		got.AngleIncrement != want.AngleIncr {
		t.Errorf("angles round-tripped as %v/%v/%v, want %v/%v/%v",
			got.AngleMin, got.AngleMax, got.AngleIncrement,
			want.AngleMin, want.AngleMax, want.AngleIncr)
	}
	if len(got.RangesM) != len(want.Ranges) {
		t.Fatalf("ranges round-tripped as %d values, want %d", len(got.RangesM), len(want.Ranges))
	}
	for i := range want.Ranges {
		if got.RangesM[i] != want.Ranges[i] {
			t.Errorf("range %d = %v, want %v", i, got.RangesM[i], want.Ranges[i])
		}
	}
}

// TestScanToProto_EmptyScanHasNoInfiniteRange guards the degenerate case:
// range_min starts at +Inf while scanning for the minimum, and an empty
// sweep would ship that Inf into the bag, where a viewer's colour ramp
// collapses.
func TestScanToProto_EmptyScanHasNoInfiniteRange(t *testing.T) {
	t.Parallel()

	got := scanToCDR(controllers.LidarScan{}, 0)
	if math.IsInf(float64(got.RangeMin), 0) || math.IsInf(float64(got.RangeMax), 0) {
		t.Errorf("empty scan produced RangeMin=%v RangeMax=%v, want finite",
			got.RangeMin, got.RangeMax)
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
	rec, err := newSimRecorder(dir, "open_0042", recorderGeometry{WheelRadiusM: 0.035, MaxSteerRad: 1.0})
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
		if writeErr := rec.run.WriteROS2(
			scanTopic, recording.LaserScanType, recording.LaserScanSchema,
			recording.EncodeLaserScan(scanToCDR(scan, logTime)), logTime,
		); writeErr != nil {
			t.Fatalf("writing scan: %v", writeErr)
		}
		if writeErr := rec.run.WriteROS2(
			navDebugTopic, recording.StringType, recording.StringSchema,
			recording.EncodeString(`{"phase":"normal_drive"}`), logTime,
		); writeErr != nil {
			t.Fatalf("writing nav debug: %v", writeErr)
		}
		rec.simClockNanos += uint64(dt * nanosPerSecond)
	}
	if closeErr := rec.close(); closeErr != nil {
		t.Fatalf("close: %v", closeErr)
	}

	// The run directory and bag are named for the SCENARIO, not a timestamp:
	// a sweep starts hundreds of runs inside one second.
	bag := filepath.Join(dir, "open_0042", "open_0042_0.mcap")
	f, err := os.Open(bag)
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
	// EVERY topic is ROS2 CDR. rosbag2 refuses to open a bag whose topics
	// do not share one serialization format, so a single protobuf channel
	// would make the whole bag unreadable by the Python diag suite.
	for topic, wantEncoding := range map[string]string{
		scanTopic:     recording.ROS2MessageEncoding,
		navDebugTopic: recording.ROS2MessageEncoding,
	} {
		encoding, ok := topics[topic]
		if !ok {
			t.Errorf("bag has no channel for %q", topic)
			continue
		}
		if encoding != wantEncoding {
			t.Errorf("channel %q encoding = %q, want %q", topic, encoding, wantEncoding)
		}
	}
	// Sim time, not wall time: four ticks at 50 ms span 150 ms end to end.
	spanNanos := info.Statistics.MessageEndTime - info.Statistics.MessageStartTime
	if want := uint64((ticks - 1) * dt * nanosPerSecond); spanNanos != want {
		t.Errorf("bag spans %d ns, want %d (the simulation clock, not the wall clock)",
			spanNanos, want)
	}
}
