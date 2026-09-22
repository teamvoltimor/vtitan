// Package natscamera reads capture frames from a NATS subject instead of a
// camera device, implementing camera.Driver.
//
// It lives here rather than in pkg/driver/camera because a driver must
// not know about the transport: that rule is what lets the drivers sit in
// pkg/driver at all (adr:0094-pkg-is-the-public-surface), and the camera
// package was the one place still breaking it. Moving it also removed the
// BindConn type assertion its caller needed, since the connection is now a
// constructor argument.
package natscamera

import (
	"context"
	"fmt"

	natsio "github.com/nats-io/nats.go"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/pkg/driver/camera"
	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"
)

// Driver subscribes to a CameraFrame topic (vtitan.sensor.v1.camera)
// and returns each frame as it arrives. This is the Go port of the Python
// VisionNode's camera_source="topic" path: the ROS2 /camera/image_raw bridge
// publishes here, so capture-node can record without touching the CSI device.
// Pure Go; builds everywhere.
type Driver struct {
	cfg camera.Config
	sub *nats.Subscriber[*sensorv1.CameraFrame]
}

var _ camera.Driver = (*Driver)(nil)

// New subscribes to cfg.NATSSubject over conn and returns the driver. The
// connection is a constructor argument rather than something bound afterwards:
// a topic-backed camera cannot exist without one, so there is no valid
// half-built state to represent.
func New(cfg camera.Config, conn *natsio.Conn) (*Driver, error) {
	sub, err := nats.NewSubscriber[sensorv1.CameraFrame](conn, cfg.NATSSubject)
	if err != nil {
		return nil, fmt.Errorf("natscamera: subscribing to %s: %w", cfg.NATSSubject, err)
	}
	return &Driver{cfg: cfg, sub: sub}, nil
}

// Open satisfies camera.Driver. The subscription is established in New, so
// there is nothing left to do here.
func (d *Driver) Open(_ context.Context) error { return nil }

// CaptureFrame blocks for the next CameraFrame on the topic.
func (d *Driver) CaptureFrame(ctx context.Context) (*camera.Frame, error) {
	msg, err := d.sub.Read(ctx)
	if err != nil {
		return nil, fmt.Errorf("natscamera: reading frame: %w", err)
	}
	return &camera.Frame{
		Width:    int(msg.GetWidth()),
		Height:   int(msg.GetHeight()),
		Stride:   int(msg.GetStride()),
		Encoding: msg.GetEncoding(),
		Data:     msg.GetData(),
	}, nil
}

// Close unsubscribes.
func (d *Driver) Close() error {
	if d.sub == nil {
		return nil
	}
	if err := d.sub.Close(); err != nil {
		return fmt.Errorf("natscamera: closing subscription: %w", err)
	}
	return nil
}
