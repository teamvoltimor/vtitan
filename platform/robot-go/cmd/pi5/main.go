// Command pi5 is the production combined board binary for the Pi 5: IMU +
// LIDAR + vision + telemetry + state-machine + track-navigator, run as
// supervised goroutines in a single process. See
// platform/robot/docs/internal/plans/go-migration-plan.md ("Process model").
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/pi5: not yet implemented")
	os.Exit(1)
}
