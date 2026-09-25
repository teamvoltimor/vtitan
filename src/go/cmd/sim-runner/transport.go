package main

import (
	"github.com/spf13/pflag"

	"github.com/teamvoltimor/vtitan/src/go/internal/sim/harness"
)

// registerTransportFlags adds the transport emulation flags.
func registerTransportFlags(flags *pflag.FlagSet, cfg *cliConfig) {
	// Transport: the time and commands lost between the navigator and the
	// body and sensors, which the in-process sim otherwise treats as zero.
	// All default to zero, the condition of every existing corpus number.
	flags.Float64Var(
		&cfg.cmdDelayS,
		"command-delay-s",
		0,
		"--runner native only: delay from a published drive command to the body (NATS, link and board together)",
	)
	flags.Float64Var(
		&cfg.cmdDropRate,
		"command-drop-rate",
		0,
		"--runner native only: probability a drive command never arrives; the previous one keeps applying",
	)
	flags.Float64Var(
		&cfg.cmdTimeoutS,
		"command-timeout-s",
		0,
		"--runner native only: emulated board watchdog, stop and centre after this long without a command; 0 is off",
	)
	flags.Float64Var(
		&cfg.scanDelayS,
		"scan-delay-s",
		0,
		"--runner native only: age of the LIDAR sweep the navigator reads",
	)
}

func transportFor(cfg cliConfig) harness.TransportConfig {
	return harness.TransportConfig{
		CommandDelayS:   cfg.cmdDelayS,
		CommandDropRate: cfg.cmdDropRate,
		CommandTimeoutS: cfg.cmdTimeoutS,
		ScanDelayS:      cfg.scanDelayS,
	}
}
