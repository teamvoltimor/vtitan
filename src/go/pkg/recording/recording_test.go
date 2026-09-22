package recording

import (
	"bytes"
	"encoding/binary"
	"os"
	"path/filepath"
	"strings"
	"testing"
	"time"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

// rgbFrame builds a tiny solid-color rgb8 frame.
func rgbFrame(w, h int, v uint8) *Frame {
	data := make([]byte, w*h*3)
	for i := range data {
		data[i] = v
	}
	return &Frame{Width: w, Height: h, Stride: w * 3, Encoding: "rgb8", Data: data}
}

func TestMJPEGAVIWriteAndClose(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	path := filepath.Join(dir, "video.mp4")

	sink, err := newMJPEGAVISink(path)
	if err != nil {
		t.Fatalf("newMJPEGAVISink: %v", err)
	}
	for i := range 3 {
		if err = sink.Write(rgbFrame(16, 12, uint8(i*40+30)), HudOverlay{}); err != nil {
			t.Fatalf("Write %d: %v", i, err)
		}
	}
	if err = sink.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}

	raw, err := os.ReadFile(path)
	if err != nil {
		t.Fatalf("read: %v", err)
	}
	if len(raw) < 12 {
		t.Fatalf("avi too small: %d bytes", len(raw))
	}
	if !bytes.HasPrefix(raw, []byte("RIFF")) {
		t.Errorf("avi missing RIFF header, got %q", raw[:4])
	}
	if !bytes.Contains(raw, []byte("AVI ")) {
		t.Errorf("avi missing AVI list")
	}
	if !bytes.Contains(raw, []byte("movi")) {
		t.Errorf("avi missing movi list")
	}
}

func TestPhotoCaptureCadence(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	pc := NewPhotoCapture(10*time.Second, "captures", false)

	// First call saves regardless of elapsed time (lastCapture is zero).
	p, err := pc.MaybeCapture(time.Unix(0, 0), rgbFrame(4, 4, 10), dir, false)
	if err != nil || p == "" {
		t.Fatalf("first capture: p=%q err=%v", p, err)
	}
	if _, err = os.Stat(p); err != nil {
		t.Fatalf("capture file missing: %v", err)
	}
	// Immediate second call within cadence: skipped.
	if p2, _ := pc.MaybeCapture(time.Unix(1, 0), rgbFrame(4, 4, 20), dir, false); p2 != "" {
		t.Errorf("expected skip within cadence, got %q", p2)
	}
	// After interval: saves again.
	p3, err := pc.MaybeCapture(time.Unix(11, 0), rgbFrame(4, 4, 30), dir, false)
	if err != nil || p3 == "" {
		t.Fatalf("second capture: p=%q err=%v", p3, err)
	}
}

func TestPhotoCaptureRequireDetection(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	pc := NewPhotoCapture(0, "captures", true) // require detection, no interval
	if p, _ := pc.MaybeCapture(time.Unix(0, 0), rgbFrame(2, 2, 1), dir, false); p != "" {
		t.Errorf("should skip without detection, got %q", p)
	}
	if p, err := pc.MaybeCapture(time.Unix(0, 0), rgbFrame(2, 2, 1), dir, true); err != nil || p == "" {
		t.Fatalf("should save with detection: p=%q err=%v", p, err)
	}
}

func TestRunRecorderMCAPRoundTrip(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	rec, err := NewRun(dir, RunOptions{Video: false})
	if err != nil {
		t.Fatalf("NewRun: %v", err)
	}
	msg := &sensorv1.CameraFrame{Width: 8, Height: 6, Encoding: "rgb8", Data: []byte("fake")}
	if err = rec.WriteMessage(sensorv1.CameraSubject, msg, 123); err != nil {
		t.Fatalf("WriteMessage: %v", err)
	}
	if err = rec.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	// The bag should exist and be non-empty.
	bag := filepath.Join(rec.Dir(), "run_"+rec.stamp+"_0.mcap")
	fi, err := os.Stat(bag)
	if err != nil {
		t.Fatalf("bag missing: %v", err)
	}
	if fi.Size() == 0 {
		t.Errorf("bag is empty")
	}
}

// TestEncodePath_LaysOutTheNestedHeaders guards the shape a positional
// format makes fragile: nav_msgs/Path is a header followed by a COUNT and
// then that many PoseStamped, each of which carries its OWN header. Omitting
// the inner headers still produces a buffer of plausible length, and every
// field after the first pose lands in the wrong place -- a plan drawn as
// scattered points rather than a route.
//
// Checked by walking the bytes here; the authoritative check is that the
// real rclpy nav_msgs/Path deserializer reads it back, which is how this
// encoding was validated when written.
func TestEncodePath_LaysOutTheNestedHeaders(t *testing.T) {
	t.Parallel()

	const frame = "map"
	points := []PathPointCDR{{X: 1, Y: 2}, {X: 3, Y: 4}}
	raw := EncodePath(PathCDR{
		FrameID: frame, StampSec: 1, StampNanosec: 2, Points: points,
	})

	// 4 encapsulation + header(4 sec + 4 nsec + 4 len + 4 "map\0") = 20,
	// then the uint32 pose count.
	const headerEnd = 20
	if len(raw) < headerEnd+4 {
		t.Fatalf("encoded Path is %d bytes, too short to hold a header and a count", len(raw))
	}
	count := binary.LittleEndian.Uint32(raw[headerEnd : headerEnd+4])
	if int(count) != len(points) {
		t.Errorf("pose count = %d, want %d", count, len(points))
	}
	// A pose WITH its own header costs materially more than one without:
	// the bare Pose is 7 float64 = 56 bytes, and the header adds a stamp and
	// a frame string on top. Measuring the per-pose GROWTH rather than
	// restating the byte layout keeps this a check on the encoder instead of
	// a copy of it -- and 56 is exactly what the buggy version would give.
	const barePoseBytes = 7 * 8
	three := EncodePath(PathCDR{
		FrameID: frame, StampSec: 1, StampNanosec: 2,
		Points: []PathPointCDR{{X: 1, Y: 2}, {X: 3, Y: 4}, {X: 5, Y: 6}},
	})
	if growth := len(three) - len(raw); growth <= barePoseBytes {
		t.Errorf("one more pose added %d bytes, want more than %d -- each pose must carry its own header",
			growth, barePoseBytes)
	}
	if !strings.Contains(string(raw), frame) {
		t.Errorf("encoded Path does not contain the frame id %q", frame)
	}
}

// TestWriteMetadata_MakesTheRunDirectoryABag checks the rosbag2 sidecar,
// which is what lets a diag_bag_*.py script take the run DIRECTORY the way
// it does for a pulled hardware bag. Without it rosbag2 reports "No storage
// could be initialized for the input URI" and every script has to be handed
// the .mcap path instead.
func TestWriteMetadata_MakesTheRunDirectoryABag(t *testing.T) {
	t.Parallel()

	dir := t.TempDir()
	run, err := NewRun(dir, RunOptions{Name: "open_0007"})
	if err != nil {
		t.Fatalf("NewRun: %v", err)
	}
	const oneSecond = 1_000_000_000
	for i, logTime := range []uint64{0, oneSecond / 2, oneSecond} {
		if err = run.WriteROS2(
			"/motor/drive_speed", Float32Type, Float32Schema, EncodeFloat32(float32(i)), logTime,
		); err != nil {
			t.Fatalf("WriteROS2: %v", err)
		}
	}
	if err = run.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}

	raw, err := os.ReadFile(filepath.Join(dir, "open_0007", "metadata.yaml"))
	if err != nil {
		t.Fatalf("reading metadata.yaml: %v", err)
	}
	meta := string(raw)

	for _, want := range []string{
		"storage_identifier: mcap",
		"name: /motor/drive_speed",
		"type: " + Float32Type,
		"serialization_format: cdr",
		// Required by the v9 schema: rosbag2 rejects the entire file when
		// this key is missing, which is how it was found.
		"type_description_hash:",
		// The bag file must be named as a RELATIVE path, or rosbag2 looks
		// for it in the wrong place.
		"- open_0007_0.mcap",
		"message_count: 3",
	} {
		if !strings.Contains(meta, want) {
			t.Errorf("metadata.yaml is missing %q:\n%s", want, meta)
		}
	}
	// The span is measured from the messages, not the wall clock: these
	// three were logged across one simulated second.
	if !strings.Contains(meta, "nanoseconds: 1000000000") {
		t.Errorf("metadata.yaml does not report the 1 s message span:\n%s", meta)
	}
}
