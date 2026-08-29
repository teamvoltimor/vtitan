// Command pi-zero is the production combined board binary for the Pi Zero:
// motor + button + OLED, run as supervised goroutines in a single process.
// See platform/robot/docs/internal/plans/go-migration-plan.md ("Process model").
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/pi-zero: not yet implemented")
	os.Exit(1)
}
