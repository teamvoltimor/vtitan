// Package camera defines the capture-driver contract used by the run recorder
// and capture-node. It deliberately hides the sensor specifics (CSI camera
// module 3, a ROS2 /camera/image_raw bridge, a test pattern) behind one
// Driver interface, so the recorder and capture-node depend only on this
// interface -- never on a concrete sensor. The concrete backend is chosen at
// startup from config (robot.toml [camera].source), mirroring the Python
// VisionNode's camera_source split (config/hardware/vision/node.toml).
//
// Only the "v4l2" backend (internal/driver/camera/gocv) links OpenCV and is
// built behind the `cgo` build tag; "topic" and "synthetic" are pure Go.
package camera

import (
	"context"
	"errors"
	"fmt"
)

// errNoNATSConn is returned by NATSSourceDriver.Open when BindConn was not
// called with a live connection first.
var errNoNATSConn = errors.New("camera: NATSSourceDriver used before BindConn")

// Config selects and parameterizes a capture backend. It is populated from
// robot.toml's [camera] section (see config/profile.RobotCamera); nothing here
// is hardcoded to a specific sensor.
type Config struct {
	// Source is "v4l2" | "topic" | "synthetic".
	Source string
	// Device is the V4L2 node for the "v4l2" backend (e.g. "/dev/video0").
	Device string
	Width  int
	Height int
	// FPS is the capture rate; the recorder uses it to size the video encoder.
	FPS float64
	// NATSSubject is the topic the "topic" backend subscribes to.
	NATSSubject string
}

// Source constants. Keep in sync with robot.toml [camera].source values.
const (
	SourceV4L2      = "v4l2"
	SourceTopic     = "topic"
	SourceSynthetic = "synthetic"
)

// Frame is one captured image. Data is contiguous rows of Width x Height
// pixels, Stride bytes per row, in the channel order Encoding names (e.g.
// "rgb8"). It matches the wire shape of sensorv1.CameraFrame so a frame pulled
// off the NATS "topic" backend and one grabbed from the CSI device are
// interchangeable downstream.
type Frame struct {
	Width    int
	Height   int
	Stride   int
	Encoding string
	Data     []byte
}

// Driver captures frames from a concrete source. Implementations must be safe
// for concurrent use of CaptureFrame after Open returns (the recorder's capture
// loop is the only caller, so a single goroutine is the common case).
type Driver interface {
	// Open prepares the source. It must be called before CaptureFrame.
	Open(ctx context.Context) error
	// CaptureFrame blocks until the next frame is available or ctx is done.
	CaptureFrame(ctx context.Context) (*Frame, error)
	// Close releases the source.
	Close() error
}

// New constructs the Driver named by cfg.Source.
func New(cfg Config) (Driver, error) {
	switch cfg.Source {
	case SourceSynthetic, "":
		return NewSynthetic(cfg), nil
	case SourceTopic:
		return NewNATSSource(cfg), nil
	case SourceV4L2:
		return newV4L2Driver(cfg)
	default:
		return nil, fmt.Errorf("camera: unknown source %q (want v4l2|topic|synthetic)", cfg.Source)
	}
}
