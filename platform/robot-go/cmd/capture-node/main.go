// Command capture-node records a robot run's video + periodic photos from a
// camera, starting on process boot -- matching the Python robot's "record from
// power-on" behavior. The actual capture loop lives in internal/node/capture so
// cmd/pi5 can run it as a supervised goroutine too.
//
// The camera backend is chosen from --source (robot.toml [camera].source):
//   - v4l2:   direct CSI capture via gocv (built under the `cgo` tag only)
//   - topic:  subscribe to vtitan.sensor.v1.camera (ROS2 /camera/image_raw bridge)
//   - synthetic: test pattern, for bench/sim without hardware
package main

import (
	"context"
	"flag"
	"log/slog"
	"os"
	"os/signal"
	"syscall"
	"time"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/camera"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/node/capture"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stderr, nil))

	var (
		source        = flag.String("source", camera.SourceSynthetic, "capture backend: v4l2|topic|synthetic")
		device        = flag.String("device", "/dev/video0", "V4L2 device node (v4l2 backend)")
		natsURL       = flag.String("nats-url", nats.DefaultDevURL, "nats-server URL (topic backend)")
		nodeName      = flag.String("name", "capture-node", "NATS client name")
		runsRoot      = flag.String("runs-root", "", "runs root dir (default: repo-root data/runs_pulled)")
		fps           = flag.Float64("fps", 15.0, "capture/video frame rate")
		photoInterval = flag.Duration("photo-interval", 10*time.Second, "periodic dataset-photo cadence (0 = off)")
		photoSubdir   = flag.String("photo-subdir", "captures", "subdir under run dir for photos")
		requireDet    = flag.Bool("require-detection", false, "only save photos when a detection is present (Obstacles)")
		video         = flag.Bool("video", true, "record the debug video")
	)
	flag.Parse()

	ctx, stop := signal.NotifyContext(context.Background(), os.Interrupt, syscall.SIGTERM)
	defer stop()

	cfg := capture.Config{
		Camera: camera.Config{
			Source:      *source,
			Device:      *device,
			FPS:         *fps,
			NATSSubject: "vtitan.sensor.v1.camera",
		},
		NATS:            nats.DefaultConfig(*natsURL, *nodeName),
		RunsRoot:        *runsRoot,
		FPS:             *fps,
		Video:           *video,
		PhotoInterval:   *photoInterval,
		PhotoSubdir:     *photoSubdir,
		PhotoRequireDet: *requireDet,
	}

	if err := capture.Run(ctx, cfg, logger); err != nil && err != context.Canceled {
		logger.Error("capture-node: run failed", "error", err)
		os.Exit(1)
	}
	os.Exit(0)
}
