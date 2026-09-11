package camera

import (
	"context"

	natsio "github.com/nats-io/nats.go"

	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
)

// NATSSourceDriver subscribes to a CameraFrame topic (vtitan.sensor.v1.camera)
// and returns each frame as it arrives. This is the Go port of the Python
// VisionNode's camera_source="topic" path: the ROS2 /camera/image_raw bridge
// publishes here, so capture-node can record without touching the CSI device.
// Pure Go; builds everywhere.
type NATSSourceDriver struct {
	cfg Config
	sub *nats.Subscriber[*sensorv1.CameraFrame]
}

// NewNATSSource builds a topic-backed capture driver. The connection must be
// supplied via the Config's NATSSubject + a separately-established conn; this
// constructor returns the driver and Open wires the subscription.
func NewNATSSource(cfg Config) *NATSSourceDriver {
	return &NATSSourceDriver{cfg: cfg}
}

// Open establishes the NATS subscription. It returns an error if no connection
// was bound (see BindConn) -- kept separate from New so construction never
// touches the network.
func (d *NATSSourceDriver) Open(_ context.Context) error {
	if d.sub == nil {
		return errNoNATSConn
	}
	return nil
}

// BindConn attaches a NATS subscription to the configured subject. Called once
// before Open, after a connection is established.
func (d *NATSSourceDriver) BindConn(conn *natsio.Conn) error {
	sub, err := nats.NewSubscriber(conn, d.cfg.NATSSubject, func() *sensorv1.CameraFrame {
		return &sensorv1.CameraFrame{}
	})
	if err != nil {
		return err
	}
	d.sub = sub
	return nil
}

// CaptureFrame blocks for the next CameraFrame on the topic.
func (d *NATSSourceDriver) CaptureFrame(ctx context.Context) (*Frame, error) {
	msg, err := d.sub.Read(ctx)
	if err != nil {
		return nil, err
	}
	return &Frame{
		Width:    int(msg.GetWidth()),
		Height:   int(msg.GetHeight()),
		Stride:   int(msg.GetStride()),
		Encoding: msg.GetEncoding(),
		Data:     msg.GetData(),
	}, nil
}

// Close unsubscribes.
func (d *NATSSourceDriver) Close() error {
	if d.sub == nil {
		return nil
	}
	return d.sub.Close()
}
