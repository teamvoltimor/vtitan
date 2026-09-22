package recording

import (
	"fmt"
	"image"
	"image/color"
	"image/jpeg"
	"os"
	"path/filepath"
	"sync"
	"time"
)

// PhotoCapture saves raw (un-annotated) frames to <run_dir>/<subdir>/ at most
// once every Interval. It is the Go port of Python's dataset_capture.py: the
// same cadence + detection gate, with the same "bag recorder owns the run dir,
// this only polls for it" rule (dataset_capture.py:64).
type PhotoCapture struct {
	mu          sync.Mutex
	interval    time.Duration
	subdir      string
	lastCapture time.Time
	count       int
	requireDet  bool
}

// rgb8Image adapts an rgb8 Frame to image.Image without copying the buffer.
type rgb8Image struct{ frame *Frame }

const (
	// photoJPEGQuality is the JPEG quality periodic captures encode at.
	photoJPEGQuality = 90
	// rgb8BytesPerPixel is the byte count of one contiguous rgb8 pixel.
	rgb8BytesPerPixel = 3
	// rgb8MaxChannel is the full-scale value of one 8-bit channel.
	rgb8MaxChannel = 255
)

// NewPhotoCapture builds a periodic capture. If requireDetection is true, a
// frame is only saved once a detection is present (Obstacles Challenge); Open
// Challenge passes false to save every interval unconditionally.
func NewPhotoCapture(interval time.Duration, subdir string, requireDetection bool) *PhotoCapture {
	return &PhotoCapture{
		interval:    interval,
		subdir:      subdir,
		requireDet:  requireDetection,
		lastCapture: time.Time{}, // zero = never, so the first eligible frame saves
	}
}

// MaybeCapture saves frame if the cadence and (when required) detection gate
// both allow it. runDir may be empty (no run in progress) -- then it is a
// no-op, matching dataset_capture.py:56. Returns the saved path or "" if no
// capture occurred.
func (p *PhotoCapture) MaybeCapture(now time.Time, frame *Frame, runDir string, hasDetection bool) (string, error) {
	p.mu.Lock()
	defer p.mu.Unlock()

	if runDir == "" {
		return "", nil
	}
	if p.interval > 0 && now.Sub(p.lastCapture) < p.interval {
		return "", nil
	}
	if p.requireDet && !hasDetection {
		return "", nil
	}

	dir := filepath.Join(runDir, p.subdir)
	if err := os.MkdirAll(dir, dirMode); err != nil {
		return "", fmt.Errorf("recording: mkdir captures dir: %w", err)
	}
	filename := filepath.Join(dir, fmt.Sprintf("capture_%04d.jpg", p.count))
	if err := writeJPEG(filename, frame); err != nil {
		return "", err
	}
	p.count++
	p.lastCapture = now
	return filename, nil
}

// Reset clears per-run state. Call on each RACING start edge, matching
// dataset_capture.py:reset (restarts filenames from capture_0000).
func (p *PhotoCapture) Reset() {
	p.mu.Lock()
	defer p.mu.Unlock()
	p.lastCapture = time.Time{}
	p.count = 0
}

// writeJPEG encodes an rgb8 Frame as a JPEG. Go's jpeg encoder takes RGBA
// directly, so this matches dataset_capture.py's rgb->bgr then cv2.imwrite.
func writeJPEG(path string, frame *Frame) error {
	if frame.Encoding != EncodingRGB8 {
		return fmt.Errorf("recording: unsupported photo encoding %q (want %s)", frame.Encoding, EncodingRGB8)
	}
	img, err := rgb8ToImage(frame)
	if err != nil {
		return err
	}
	f, err := os.Create(path)
	if err != nil {
		return fmt.Errorf("recording: creating %s: %w", path, err)
	}
	defer f.Close()
	if err = jpeg.Encode(f, img, &jpeg.Options{Quality: photoJPEGQuality}); err != nil {
		return fmt.Errorf("recording: encoding jpeg %s: %w", path, err)
	}
	return nil
}

// rgb8ToImage wraps a contiguous rgb8 buffer as an image.Image.
func rgb8ToImage(frame *Frame) (image.Image, error) {
	if frame.Stride != frame.Width*rgb8BytesPerPixel {
		return nil, fmt.Errorf(
			"recording: rgb8 stride %d != width*%d %d",
			frame.Stride,
			rgb8BytesPerPixel,
			frame.Width*rgb8BytesPerPixel,
		)
	}
	if len(frame.Data) < frame.Height*frame.Stride {
		return nil, fmt.Errorf(
			"recording: rgb8 data len %d < height*stride %d",
			len(frame.Data),
			frame.Height*frame.Stride,
		)
	}
	return &rgb8Image{frame: frame}, nil
}

func (m *rgb8Image) ColorModel() color.Model {
	return color.RGBAModel
}

func (m *rgb8Image) Bounds() image.Rectangle {
	return image.Rect(0, 0, m.frame.Width, m.frame.Height)
}

func (m *rgb8Image) At(x, y int) color.Color {
	off := y*m.frame.Stride + x*rgb8BytesPerPixel
	d := m.frame.Data
	return color.RGBA{R: d[off], G: d[off+1], B: d[off+2], A: rgb8MaxChannel}
}
