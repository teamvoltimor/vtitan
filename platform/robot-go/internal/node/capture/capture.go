// Package capture runs the per-run video + photo capture loop, shared by
// cmd/capture-node (standalone) and cmd/pi5 (as a supervised goroutine). It
// opens a camera.Driver, records into a recording.RunRecorder, and on each tick
// submits a frame to the video encoder and (optionally) saves a dataset photo.
package capture

import (
	"context"
	"errors"
	"log/slog"
	"time"

	natsio "github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/driver/camera"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/recording"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// Config configures the capture loop.
type Config struct {
	Camera          camera.Config
	NATS            nats.Config
	RunsRoot        string
	FPS             float64
	Video           bool
	PhotoInterval   time.Duration
	PhotoSubdir     string
	PhotoRequireDet bool
}

// Run opens the run, opens the camera, and loops capturing until ctx is done or
// the camera errors. It starts recording on entry (matching the Python robot's
// "record from power-on" behavior) and finalizes the run on return.
func Run(ctx context.Context, cfg Config, logger *slog.Logger) error {
	runsRoot := cfg.RunsRoot
	if runsRoot == "" {
		root, err := recording.RunsRoot()
		if err != nil {
			return err
		}
		runsRoot = root
	}

	rec, err := recording.NewRun(runsRoot, recording.RunOptions{
		Video:           cfg.Video,
		VideoFPS:        cfg.FPS,
		PhotoInterval:   cfg.PhotoInterval,
		PhotoSubdir:     cfg.PhotoSubdir,
		PhotoRequireDet: cfg.PhotoRequireDet,
	})
	if err != nil {
		return err
	}
	if err = rec.Open(); err != nil {
		return err
	}
	defer func() {
		if closeErr := rec.Close(); closeErr != nil {
			logger.Error("capture: closing run", "error", closeErr)
		}
	}()
	logger.Info("capture: recording run", "dir", rec.Dir())

	drv, err := buildDriver(ctx, cfg, logger)
	if err != nil {
		return err
	}
	if err = drv.Open(ctx); err != nil {
		return err
	}
	defer drv.Close()

	ticker := time.NewTicker(framePeriod(cfg.FPS))
	defer ticker.Stop()

	logger.Info("capture: capturing", "source", cfg.Camera.Source, "fps", cfg.FPS)
	for {
		select {
		case <-ctx.Done():
			return ctx.Err()
		case <-ticker.C:
			frame, capErr := drv.CaptureFrame(ctx)
			if capErr != nil {
				if errors.Is(capErr, context.Canceled) {
					return nil
				}
				logger.Error("capture: capture", "error", capErr)
				continue
			}
			rf := &recording.Frame{
				Width:    frame.Width,
				Height:   frame.Height,
				Stride:   frame.Stride,
				Encoding: frame.Encoding,
				Data:     frame.Data,
			}
			if rec.Video() != nil {
				rec.Video().Submit(rf, recording.HudOverlay{})
			}
			if _, phErr := rec.Photos().MaybeCapture(time.Now(), rf, rec.Dir(), false); phErr != nil {
				logger.Error("capture: photo", "error", phErr)
			}
		}
	}
}

func framePeriod(fps float64) time.Duration {
	if fps <= 0 {
		fps = 15
	}
	return time.Duration(float64(time.Second) / fps)
}

// buildDriver constructs the camera driver, wiring a NATS connection for the
// topic backend.
func buildDriver(ctx context.Context, cfg Config, logger *slog.Logger) (camera.Driver, error) {
	if cfg.Camera.Source != camera.SourceTopic {
		return camera.New(cfg.Camera)
	}
	conn, err := nats.Connect(cfg.NATS)
	if err != nil {
		return nil, err
	}
	drv, err := camera.New(cfg.Camera)
	if err != nil {
		conn.Close()
		return nil, err
	}
	binder, ok := drv.(interface{ BindConn(*natsio.Conn) error })
	if !ok {
		conn.Close()
		return nil, errors.New("capture: topic driver does not support BindConn")
	}
	if err = binder.BindConn(conn); err != nil {
		conn.Close()
		return nil, err
	}
	go func() {
		<-ctx.Done()
		conn.Close()
	}()
	logger.Info("capture: topic backend bound", "subject", sensorv1.CameraSubject)
	return drv, nil
}
