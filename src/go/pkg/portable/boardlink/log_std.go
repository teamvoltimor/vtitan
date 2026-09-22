//go:build !tinygo

package boardlink

import (
	"fmt"
	"log/slog"
)

// This file is the Go-only half of the package: the host logs packets with
// log/slog, which the firmware neither needs nor can afford. The *_std.go
// suffix exempts it from the portable-must-build-on-tinygo lint rule, and the
// build tag keeps it out of every TinyGo build.

// String names a message type, for logs.
func (t Type) String() string {
	switch t {
	case TypeHello:
		return "hello"
	case TypeConfig:
		return "config"
	case TypeCommand:
		return "command"
	case TypePing:
		return "ping"
	case TypePong:
		return "pong"
	case TypeStatus:
		return "status"
	case TypeOdometry:
		return "odometry"
	case TypeButton:
		return "button"
	}
	return fmt.Sprintf("type(0x%02x)", uint8(t))
}

// String names a board state, for logs.
func (s State) String() string {
	switch s {
	case StateUnconfigured:
		return "unconfigured"
	case StateIdle:
		return "idle"
	case StateRunning:
		return "running"
	case StateFault:
		return "fault"
	}
	return fmt.Sprintf("state(%d)", uint8(s))
}

// LogValue renders only the message Type names, so a logged Packet shows
// what was on the wire rather than eight mostly-zero structs.
func (p *Packet) LogValue() slog.Value {
	attrs := []slog.Attr{slog.String("type", p.Type.String()), slog.Int("seq", int(p.Seq))}
	switch p.Type {
	case TypeHello:
		attrs = append(attrs, slog.Any("hello", p.Hello))
	case TypeConfig:
		attrs = append(attrs, slog.Any("config", p.Config))
	case TypeCommand:
		attrs = append(attrs, slog.Any("command", p.Command))
	case TypePing:
		attrs = append(attrs, slog.Any("ping", p.Ping))
	case TypePong:
		attrs = append(attrs, slog.Any("pong", p.Pong))
	case TypeStatus:
		attrs = append(attrs, slog.String("state", p.Status.State.String()), slog.Any("status", p.Status))
	case TypeOdometry:
		attrs = append(attrs, slog.Any("odometry", p.Odometry))
	case TypeButton:
		attrs = append(attrs, slog.Bool("pressed", p.Button.Pressed))
	}
	return slog.GroupValue(attrs...)
}
