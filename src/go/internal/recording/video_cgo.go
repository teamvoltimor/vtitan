//go:build cgo && linux && arm64

package recording

import (
	"fmt"

	"gocv.io/x/gocv"
)

// gocvVideoSink is the hardware video backend: it encodes rgb8 frames to mp4v
// (matching Python's video_recorder.py fourcc(*"mp4v")) via gocv's VideoWriter
// -- the same in-process OpenCV encoder Python uses. Built only under the `cgo`
// tag; the pure-Go MJPEG sink (video_mjpeg.go) is the portable fallback.
type gocvVideoSink struct {
	path   string
	fps    float64
	writer *gocv.VideoWriter
	mat    gocv.Mat
}

// newGocvVideoSink opens an mp4v VideoWriter at path. Output dimensions are
// taken from the first frame's dimensions (video_recorder.py:129 derives height
// from aspect; we record at native capture size).
func newGocvVideoSink(path string, fps float64) (*gocvVideoSink, error) {
	s := &gocvVideoSink{path: path, fps: fps}
	s.mat = gocv.NewMat()
	return s, nil
}

// Write encodes f as a BGR frame (OpenCV expects BGR; rgb8 is converted). The
// writer is lazily created on the first frame so dimensions are known.
func (s *gocvVideoSink) Write(f *Frame, _ HudOverlay) error {
	if f.Encoding != "rgb8" {
		return fmt.Errorf("recording: gocv video wants rgb8, got %q", f.Encoding)
	}
	if s.writer == nil {
		wtr, err := gocv.VideoWriterFile(s.path, "mp4v", s.fps, f.Width, f.Height, true)
		if err != nil {
			return fmt.Errorf("recording: opening gocv writer %s: %w", s.path, err)
		}
		s.writer = wtr
	}
	// Wrap the rgb8 buffer as a Mat, then convert BGR->RGB for OpenCV.
	src, err := gocv.NewMatFromBytes(f.Height, f.Width, gocv.MatTypeCV8UC3, f.Data)
	if err != nil {
		return fmt.Errorf("recording: wrapping frame: %w", err)
	}
	defer src.Close()
	gocv.CvtColor(src, &s.mat, gocv.ColorRGBToBGR)
	return s.writer.Write(s.mat)
}

// Close finalizes the mp4 container.
func (s *gocvVideoSink) Close() error {
	if s.writer != nil {
		s.writer.Close()
		s.writer = nil
	}
	if s.mat.Ptr() != nil {
		s.mat.Close()
	}
	return nil
}
