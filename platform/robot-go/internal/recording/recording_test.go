package recording

import (
	"bytes"
	"os"
	"path/filepath"
	"testing"
	"time"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
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
	dir := t.TempDir()
	path := filepath.Join(dir, "video.mp4")

	sink, err := newMJPEGAVISink(path)
	if err != nil {
		t.Fatalf("newMJPEGAVISink: %v", err)
	}
	for i := 0; i < 3; i++ {
		if err := sink.Write(rgbFrame(16, 12, uint8(i*40+30)), HudOverlay{}); err != nil {
			t.Fatalf("Write %d: %v", i, err)
		}
	}
	if err := sink.Close(); err != nil {
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
	dir := t.TempDir()
	pc := NewPhotoCapture(10*time.Second, "captures", false)

	// First call saves regardless of elapsed time (lastCapture is zero).
	p, err := pc.MaybeCapture(time.Unix(0, 0), rgbFrame(4, 4, 10), dir, false)
	if err != nil || p == "" {
		t.Fatalf("first capture: p=%q err=%v", p, err)
	}
	if _, err := os.Stat(p); err != nil {
		t.Fatalf("capture file missing: %v", err)
	}
	// Immediate second call within cadence: skipped.
	if p2, _ := pc.MaybeCapture(time.Unix(1, 0), rgbFrame(4, 4, 20), dir, false); p2 != "" {
		t.Errorf("expected skip within cadence, got %q", p2)
	}
	// After interval: saves again.
	if p3, err := pc.MaybeCapture(time.Unix(11, 0), rgbFrame(4, 4, 30), dir, false); err != nil || p3 == "" {
		t.Fatalf("second capture: p=%q err=%v", p3, err)
	}
}

func TestPhotoCaptureRequireDetection(t *testing.T) {
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
	dir := t.TempDir()
	rec, err := NewRun(dir, RunOptions{Video: false})
	if err != nil {
		t.Fatalf("NewRun: %v", err)
	}
	msg := &sensorv1.CameraFrame{Width: 8, Height: 6, Encoding: "rgb8", Data: []byte("fake")}
	if err := rec.WriteMessage(sensorv1.CameraSubject, msg, 123); err != nil {
		t.Fatalf("WriteMessage: %v", err)
	}
	if err := rec.Close(); err != nil {
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
