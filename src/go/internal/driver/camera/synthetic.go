package camera

import (
	"context"
	"image/color"
	"math"
)

// SyntheticDriver emits a moving test pattern, for bench/sim runs that need a
// frame stream without any hardware or ROS2 bridge. Pure Go; builds everywhere.
type SyntheticDriver struct {
	cfg Config
	t   float64
}

// NewSynthetic builds a synthetic capture driver. If cfg.NATSSubject is set and
// a NATS connection is available, frames are echoed from that topic instead of
// generated locally -- but the common bench case just generates a pattern.
func NewSynthetic(cfg Config) *SyntheticDriver {
	return &SyntheticDriver{cfg: cfg}
}

// Open is a no-op for the synthetic source.
func (d *SyntheticDriver) Open(_ context.Context) error {
	return nil
}

// CaptureFrame returns the next test-pattern frame, advancing an internal clock
// so the pattern animates between calls (useful for verifying video/photo
// cadence without a camera).
func (d *SyntheticDriver) CaptureFrame(ctx context.Context) (*Frame, error) {
	select {
	case <-ctx.Done():
		return nil, ctx.Err()
	default:
	}

	w, h := d.cfg.Width, d.cfg.Height
	if w == 0 {
		w = 640
	}
	if h == 0 {
		h = 360
	}
	stride := w * 3
	data := make([]byte, h*stride)

	d.t += 0.1
	for y := 0; y < h; y++ {
		for x := 0; x < w; x++ {
			// Diagonal sweep + vertical bands: visibly different frame to frame.
			phase := math.Sin(float64(x)/float64(w)*math.Pi*4 + d.t)
			v := uint8(128 + 127*phase)
			off := y*stride + x*3
			data[off+0] = v
			data[off+1] = uint8(float64(y) / float64(h) * 255)
			data[off+2] = color.Gray{uint8(float64(x) / float64(w) * 255)}.Y
		}
	}
	return &Frame{Width: w, Height: h, Stride: stride, Encoding: "rgb8", Data: data}, nil
}

// Close is a no-op for the synthetic source.
func (d *SyntheticDriver) Close() error {
	return nil
}
