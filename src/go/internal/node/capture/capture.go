// Package capture runs the per-run video + photo capture loop, shared by
// cmd/capture-node (standalone) and cmd/pi5 (as a supervised goroutine). It
// opens a camera.Driver, records into a recording.RunRecorder, and on each tick
// submits a frame to the video encoder and (optionally) saves a dataset photo.
package capture

import (
	"context"
	"errors"
	"fmt"
	"log/slog"
	"time"

	natsio "github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/src/go/internal/driver/camera"
	"github.com/teamvoltimor/vtitan/src/go/internal/recording"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/src/go/internal/transport/nats"
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
	runsRoot, err := resolveRunsRoot(cfg.RunsRoot)
	if err != nil {
		return err
	}

	rec, err := openRun(runsRoot, cfg)
	if err != nil {
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
		return fmt.Errorf("capture: opening camera: %w", err)
	}
	defer drv.Close()

	return captureLoop(ctx, drv, rec, cfg, logger)
}

// resolveRunsRoot returns the configured run root or, when unset, discovers the
// repo-root data/live/runs directory.
func resolveRunsRoot(configured string) (string, error) {
	if configured != "" {
		return configured, nil
	}
	root, err := recording.RunsRoot()
	if err != nil {
		return "", fmt.Errorf("capture: resolving runs root: %w", err)
	}
	return root, nil
}

// openRun creates and opens a recorder for a fresh run.
func openRun(runsRoot string, cfg Config) (*recording.RunRecorder, error) {
	rec, err := recording.NewRun(runsRoot, recording.RunOptions{
		Video:           cfg.Video,
		VideoFPS:        cfg.FPS,
		PhotoInterval:   cfg.PhotoInterval,
		PhotoSubdir:     cfg.PhotoSubdir,
		PhotoRequireDet: cfg.PhotoRequireDet,
	})
	if err != nil {
		return nil, fmt.Errorf("capture: creating run: %w", err)
	}
	if err = rec.Open(); err != nil {
		return nil, fmt.Errorf("capture: opening run: %w", err)
	}
	return rec, nil
}

// captureLoop pumps frames from drv into rec until ctx is done. It returns nil
// on a clean context cancellation.
func captureLoop(
	ctx context.Context,
	drv camera.Driver,
	rec *recording.RunRecorder,
	cfg Config,
	logger *slog.Logger,
) error {
	ticker := time.NewTicker(framePeriod(cfg.FPS))
	defer ticker.Stop()

	logger.Info("capture: capturing", "camera_source", cfg.Camera.Source, "fps", cfg.FPS)
	for {
		select {
		case <-ctx.Done():
			return fmt.Errorf("capture: %w", ctx.Err())
		case <-ticker.C:
			if captureOnce(ctx, drv, rec, logger) {
				return nil
			}
		}
	}
}

// captureOnce captures and stores one frame. It reports true when the capture
// loop should stop because the context was canceled mid-capture.
func captureOnce(
	ctx context.Context,
	drv camera.Driver,
	rec *recording.RunRecorder,
	logger *slog.Logger,
) bool {
	frame, err := drv.CaptureFrame(ctx)
	if err != nil {
		if errors.Is(err, context.Canceled) {
			return true
		}
		logger.Error("capture: capture", "error", err)
		return false
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
	return false
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
		drv, err := camera.New(cfg.Camera)
		if err != nil {
			return nil, fmt.Errorf("capture: creating camera driver: %w", err)
		}
		return drv, nil
	}
	conn, err := nats.Connect(ctx, cfg.NATS)
	if err != nil {
		return nil, fmt.Errorf("capture: connecting to NATS: %w", err)
	}
	drv, err := camera.New(cfg.Camera)
	if err != nil {
		conn.Close()
		return nil, fmt.Errorf("capture: creating topic camera driver: %w", err)
	}
	binder, ok := drv.(interface{ BindConn(*natsio.Conn) error })
	if !ok {
		conn.Close()
		return nil, errors.New("capture: topic driver does not support BindConn")
	}
	if err = binder.BindConn(conn); err != nil {
		conn.Close()
		return nil, fmt.Errorf("capture: binding NATS connection: %w", err)
	}
	go func() {
		<-ctx.Done()
		conn.Close()
	}()
	logger.Info("capture: topic backend bound", "subject", sensorv1.CameraSubject)
	return drv, nil
}
