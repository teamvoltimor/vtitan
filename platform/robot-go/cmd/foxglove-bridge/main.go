// Command foxglove-bridge is the rviz2 replacement: a Foxglove Studio
// websocket bridge that republishes NATS/protobuf topics for visualization.
package main

import (
	"log/slog"
	"os"
)

func main() {
	logger := slog.New(slog.NewTextHandler(os.Stdout, nil))
	logger.Info("cmd/foxglove-bridge: not yet implemented")
	os.Exit(1)
}
