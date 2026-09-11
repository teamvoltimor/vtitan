//go:build cgo && linux && arm64

// Package gocv implements the "v4l2" camera backend via gocv (OpenCV), for real
// CSI capture on the Pi 5 (camera module 3 exposes a /dev/videoN V4L2 node under
// libcamera). It is built ONLY under the `cgo` build tag with a Linux/arm64 C
// cross-toolchain + OpenCV sysroot (see platform/robot-go/Taskfile.yml
// build:capture). Every other binary in the module stays CGO_ENABLED=0 static.
package gocv

import (
	"context"
	"fmt"

	"gocv.io/x/gocv"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/camera"
)

// Driver captures frames from a V4L2 device using gocv's VideoCapture, the same
// in-process OpenCV backend Python's cv2 uses (video_recorder.py). The device
// node comes from config (robot.toml [camera].device), never a hardcoded
// literal, so a different board or a simulated source overrides it.
type Driver struct {
	cfg camera.Config
	dev *gocv.VideoCapture
	img gocv.Mat
}

// NewV4L2 constructs the gocv-backed driver. Exported for the camera package's
// build-tagged v4l2_cgo.go, which calls it only when the `cgo` tag matches.
func NewV4L2(cfg camera.Config) (camera.Driver, error) {
	if cfg.Device == "" {
		return nil, fmt.Errorf("camera/v4l2: device is empty (set robot.toml [camera].device)")
	}
	return &Driver{cfg: cfg}, nil
}

// Open opens the V4L2 device. Sets the requested resolution/FPS if nonzero so
// the frame shape matches robot.toml (camera module 3 native is 1536x864@30).
func (d *Driver) Open(_ context.Context) error {
	vc, err := gocv.OpenVideoCapture(d.cfg.Device)
	if err != nil {
		return fmt.Errorf("camera/v4l2: opening %s: %w", d.cfg.Device, err)
	}
	if d.cfg.Width > 0 {
		vc.Set(gocv.VideoCaptureFrameWidth, float64(d.cfg.Width))
	}
	if d.cfg.Height > 0 {
		vc.Set(gocv.VideoCaptureFrameHeight, float64(d.cfg.Height))
	}
	if d.cfg.FPS > 0 {
		vc.Set(gocv.VideoCaptureFPS, d.cfg.FPS)
	}
	d.dev = vc
	d.img = gocv.NewMat()
	return nil
}

// CaptureFrame grabs the next frame and returns it as an rgb8 Frame. gocv yields
// BGR; we convert to RGB to match the "rgb8" wire encoding the recorder and the
// NATS "topic" backend both use.
func (d *Driver) CaptureFrame(ctx context.Context) (*camera.Frame, error) {
	select {
	case <-ctx.Done():
		return nil, ctx.Err()
	default:
	}
	if ok := d.dev.Read(&d.img); !ok {
		return nil, fmt.Errorf("camera/v4l2: read failed from %s", d.cfg.Device)
	}
	if d.img.Empty() {
		return nil, fmt.Errorf("camera/v4l2: empty frame from %s", d.cfg.Device)
	}

	bgr := d.img
	rgb := gocv.NewMat()
	defer rgb.Close()
	gocv.CvtColor(bgr, &rgb, gocv.ColorBGRToRGB)

	w := rgb.Cols()
	h := rgb.Rows()
	stride := w * 3
	data := make([]byte, h*stride)
	copy(data, rgb.ToBytes()) // rgb is contiguous HxWx3 after CvtColor
	return &camera.Frame{
		Width:    w,
		Height:   h,
		Stride:   stride,
		Encoding: "rgb8",
		Data:     data,
	}, nil
}

// Close releases the device and the scratch mat.
func (d *Driver) Close() error {
	if d.img.Ptr() != nil {
		d.img.Close()
	}
	if d.dev != nil {
		return d.dev.Close()
	}
	return nil
}
