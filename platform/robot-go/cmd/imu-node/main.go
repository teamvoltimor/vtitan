// Command imu-node is a bench/dev single-subsystem binary for the IMU
// driver, sharing the same internal packages as cmd/pi5 — used for isolated
// hardware bench testing and local debugging without the whole board binary.
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/imu-node: not yet implemented")
	os.Exit(1)
}
