// Command track-navigator is a bench/dev single-subsystem binary for the
// nav stack (track model, direction estimator, sign router, navigator),
// sharing the same internal packages as cmd/pi5 — used for isolated bench
// testing and local debugging, and for bag-replay/sim-corpus parity work
// before this subsystem is trusted inside cmd/pi5.
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/track-navigator: not yet implemented")
	os.Exit(1)
}
