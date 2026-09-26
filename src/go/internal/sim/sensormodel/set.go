// Package sensormodel ports the Python oracle's measured sensor models to
// the Go world simulator (platform plan item 2.12): the LIDAR's chassis
// bands and self-returns, the camera's detection statistics, the IMU error
// budget, control-tick jitter and the reverse-run rule 9.21.
//
// Every model ships OFF, so each Go corpus number keeps its condition until
// the models are switched on together in one re-baseline. Each model draws
// from its own RNG stream, so switching one on cannot shift another's draws.
// The values are the shipped simulation.toml ones, which carry their
// measurement provenance; Python's numpy streams cannot be reproduced in Go,
// so the port matches them statistically, not draw for draw.
package sensormodel

import (
	"fmt"
	"strings"
)

// Set is which sensor models a run switches on. The zero value is none.
type Set uint8

// The models a Set can hold.
const (
	// Lidar adds the chassis bands (dropout and sub-floor self-returns) and
	// the measured invalid-ray rate outside them.
	Lidar Set = 1 << iota
	// Vision adds the camera's range falloff, frame misses, confidence
	// spread, bearing scatter and color flips.
	Vision
	// IMU adds the IMU error budget, and with it the LIDAR localizer and
	// the wall-heading correction that bound it on the robot.
	IMU
	// TickJitter samples each control period from the measured
	// distribution instead of the nominal one.
	TickJitter
	// ReverseRun scores WRO rule 9.21: driving against the round direction
	// outside the section it began in and the one behind it ends the round.
	ReverseRun
)

// All is every model.
const All = Lidar | Vision | IMU | TickJitter | ReverseRun

// setNames are the names ParseSet accepts, in String's order.
var setNames = []struct {
	name  string
	model Set
}{
	{"lidar", Lidar},
	{"vision", Vision},
	{"imu", IMU},
	{"tick-jitter", TickJitter},
	{"reverse-run", ReverseRun},
}

// ParseSet reads a comma-separated list of model names, "all", or "" / "none"
// for no model.
func ParseSet(raw string) (Set, error) {
	var set Set
	for field := range strings.SplitSeq(raw, ",") {
		name := strings.TrimSpace(strings.ToLower(field))
		switch name {
		case "", "none":
			continue
		case "all":
			set |= All
			continue
		}
		model, ok := modelNamed(name)
		if !ok {
			return 0, fmt.Errorf("sensormodel: unknown model %q (want all, none or a list of %s)",
				name, strings.Join(Names(), ", "))
		}
		set |= model
	}
	return set, nil
}

// Names lists every model name ParseSet accepts, besides all and none.
func Names() []string {
	names := make([]string, len(setNames))
	for i, n := range setNames {
		names[i] = n.name
	}
	return names
}

// Has reports whether every model in m is on.
func (s Set) Has(m Set) bool {
	return s&m == m
}

// String is the comma-separated list ParseSet reads back.
func (s Set) String() string {
	if s == 0 {
		return "none"
	}
	var parts []string
	for _, n := range setNames {
		if s.Has(n.model) {
			parts = append(parts, n.name)
		}
	}
	return strings.Join(parts, ",")
}

func modelNamed(name string) (Set, bool) {
	for _, n := range setNames {
		if n.name == name {
			return n.model, true
		}
	}
	return 0, false
}
