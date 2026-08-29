// Command motor-node is a bench/dev single-subsystem binary for the BTS7960
// motor control loop, sharing the same internal packages as cmd/pi-zero —
// used for isolated hardware bench testing and local debugging.
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/motor-node: not yet implemented")
	os.Exit(1)
}
