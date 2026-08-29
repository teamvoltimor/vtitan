// Command lidar-node is the standalone LIDAR binary, kept independent of
// cmd/pi5 (not folded in) because per-node isolation for the LIDAR driver
// already earned its keep operationally (vtitan-lidar.service precedent).
// Also usable for bench testing, sharing internal packages with cmd/pi5.
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/lidar-node: not yet implemented")
	os.Exit(1)
}
