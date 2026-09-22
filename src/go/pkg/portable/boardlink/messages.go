package boardlink

import (
	"encoding/binary"
	"math"
)

// Type identifies a message. Values are part of the wire contract: never
// renumber one, only add.
type Type uint8

// State is the board's actuation state, reported in Status. It maps onto
// MotorStatus.State on the host, plus Unconfigured, which the Zero never has.
type State uint8

// Hello is sent by the board at boot and repeatedly until configured.
type Hello struct {
	// ProtocolVersion is the Version the firmware was built with.
	ProtocolVersion uint8
	// BootID is random per boot, so the host can tell a reset from a
	// repeated Hello.
	BootID uint32
	// Faults carries FaultWatchdogReset when the boot followed a watchdog
	// reset.
	Faults uint8
}

// Config is everything the board needs to turn a Command into actuator
// setpoints, taken by the host from its hardware profile.
type Config struct {
	CommandTimeoutMS    uint16
	SpeedScalePctPerMPS float32
	InvertDrive         bool
	LinkageRatio        float32
	ServoMaxAngleDeg    float32
	SteeringOffsetDeg   float32
	ServoMinPulseUS     float32
	ServoMaxPulseUS     float32
	ServoCenterPulseUS  float32
	ServoRangeDeg       float32
	ServoReversed       bool
	HardwareWatchdogMS  uint16
	StatusIntervalMS    uint16
	OdometryIntervalMS  uint16
}

// Command is one AckermannCmd: wheel angle and speed, in the units the
// message carries. Conversion to duty and servo angle happens on the board,
// with pkg/portable/actuation, exactly as the Zero does it.
type Command struct {
	SpeedMPS         float32
	SteeringAngleRad float32
}

// Ping carries the host's clock.
type Ping struct {
	HostTimeUS uint64
}

// Pong echoes Ping.HostTimeUS with the board's clock at receipt.
type Pong struct {
	HostTimeUS  uint64
	BoardTimeUS uint64
}

// Status is the board's periodic report.
type Status struct {
	BoardTimeUS   uint64
	State         State
	Faults        uint8
	Duty          float32
	ServoAngleDeg float32
	CommandAgeMS  uint32
}

// Odometry is the raw signed quadrature count at BoardTimeUS. The host turns
// it into distance and speed with pkg/portable/quadrature, the same code the
// Zero runs, so the two boards cannot disagree about what a count means.
type Odometry struct {
	BoardTimeUS uint64
	Counts      int64
}

// Packet is one decoded or to-be-encoded message. Exactly the field named by
// Type is meaningful. It is a struct of values rather than an interface so
// that decoding boxes nothing and allocates nothing.
type Packet struct {
	Seq      uint16
	Type     Type
	Hello    Hello
	Config   Config
	Command  Command
	Ping     Ping
	Pong     Pong
	Status   Status
	Odometry Odometry
}

// writer appends little-endian fields to a fixed buffer.
type writer struct {
	b []byte
}

// reader consumes little-endian fields from a body already checked for
// length, so it never runs short.
type reader struct {
	b []byte
	i int
}

// Message types. Host->board: Config, Command, Ping. Board->host: Hello,
// Pong, Status, Odometry.
const (
	TypeHello    Type = 0x01
	TypeConfig   Type = 0x02
	TypeCommand  Type = 0x03
	TypePing     Type = 0x04
	TypePong     Type = 0x05
	TypeStatus   Type = 0x06
	TypeOdometry Type = 0x07
)

// Board states.
const (
	StateUnconfigured State = 0
	StateIdle         State = 1
	StateRunning      State = 2
	StateFault        State = 3
)

// Fault bits, reported in Status.Faults.
const (
	// FaultWatchdogReset: the board's last reset was its hardware watchdog.
	FaultWatchdogReset uint8 = 1 << 0
	// FaultActuator: the last servo or H-bridge write failed.
	FaultActuator uint8 = 1 << 1
	// FaultRejectedCommand: a non-finite command was dropped since the last
	// Status.
	FaultRejectedCommand uint8 = 1 << 2
)

// Body sizes on the wire. Fixed per type: a length mismatch is a corrupt or
// foreign frame.
const (
	helloLen    = 1 + 4 + 1
	configLen   = 2 + 4 + 1 + 4*7 + 1 + 2 + 2 + 2
	commandLen  = 4 + 4
	pingLen     = 8
	pongLen     = 8 + 8
	statusLen   = 8 + 1 + 1 + 4 + 4 + 4
	odometryLen = 8 + 8
	maxBodyLen  = configLen
)

func bodyLen(t Type) (int, bool) {
	switch t {
	case TypeHello:
		return helloLen, true
	case TypeConfig:
		return configLen, true
	case TypeCommand:
		return commandLen, true
	case TypePing:
		return pingLen, true
	case TypePong:
		return pongLen, true
	case TypeStatus:
		return statusLen, true
	case TypeOdometry:
		return odometryLen, true
	}
	return 0, false
}

func (w *writer) u8(v uint8)    { w.b = append(w.b, v) }
func (w *writer) u16(v uint16)  { w.b = binary.LittleEndian.AppendUint16(w.b, v) }
func (w *writer) u32(v uint32)  { w.b = binary.LittleEndian.AppendUint32(w.b, v) }
func (w *writer) u64(v uint64)  { w.b = binary.LittleEndian.AppendUint64(w.b, v) }
func (w *writer) f32(v float32) { w.u32(math.Float32bits(v)) }

func (w *writer) flag(v bool) {
	if v {
		w.u8(1)
		return
	}
	w.u8(0)
}

func (r *reader) u8() uint8 {
	v := r.b[r.i]
	r.i++
	return v
}

func (r *reader) u16() uint16 {
	v := binary.LittleEndian.Uint16(r.b[r.i:])
	r.i += 2
	return v
}

func (r *reader) u32() uint32 {
	v := binary.LittleEndian.Uint32(r.b[r.i:])
	r.i += 4
	return v
}

func (r *reader) u64() uint64 {
	v := binary.LittleEndian.Uint64(r.b[r.i:])
	r.i += 8
	return v
}

func (r *reader) f32() float32 { return math.Float32frombits(r.u32()) }
func (r *reader) flag() bool   { return r.u8() != 0 }

func appendBody(w *writer, p *Packet) {
	switch p.Type {
	case TypeHello:
		w.u8(p.Hello.ProtocolVersion)
		w.u32(p.Hello.BootID)
		w.u8(p.Hello.Faults)
	case TypeConfig:
		c := &p.Config
		w.u16(c.CommandTimeoutMS)
		w.f32(c.SpeedScalePctPerMPS)
		w.flag(c.InvertDrive)
		w.f32(c.LinkageRatio)
		w.f32(c.ServoMaxAngleDeg)
		w.f32(c.SteeringOffsetDeg)
		w.f32(c.ServoMinPulseUS)
		w.f32(c.ServoMaxPulseUS)
		w.f32(c.ServoCenterPulseUS)
		w.f32(c.ServoRangeDeg)
		w.flag(c.ServoReversed)
		w.u16(c.HardwareWatchdogMS)
		w.u16(c.StatusIntervalMS)
		w.u16(c.OdometryIntervalMS)
	case TypeCommand:
		w.f32(p.Command.SpeedMPS)
		w.f32(p.Command.SteeringAngleRad)
	case TypePing:
		w.u64(p.Ping.HostTimeUS)
	case TypePong:
		w.u64(p.Pong.HostTimeUS)
		w.u64(p.Pong.BoardTimeUS)
	case TypeStatus:
		s := &p.Status
		w.u64(s.BoardTimeUS)
		w.u8(uint8(s.State))
		w.u8(s.Faults)
		w.f32(s.Duty)
		w.f32(s.ServoAngleDeg)
		w.u32(s.CommandAgeMS)
	case TypeOdometry:
		w.u64(p.Odometry.BoardTimeUS)
		w.u64(uint64(p.Odometry.Counts))
	}
}

func readBody(r *reader, p *Packet) {
	switch p.Type {
	case TypeHello:
		p.Hello = Hello{ProtocolVersion: r.u8(), BootID: r.u32(), Faults: r.u8()}
	case TypeConfig:
		p.Config = Config{
			CommandTimeoutMS:    r.u16(),
			SpeedScalePctPerMPS: r.f32(),
			InvertDrive:         r.flag(),
			LinkageRatio:        r.f32(),
			ServoMaxAngleDeg:    r.f32(),
			SteeringOffsetDeg:   r.f32(),
			ServoMinPulseUS:     r.f32(),
			ServoMaxPulseUS:     r.f32(),
			ServoCenterPulseUS:  r.f32(),
			ServoRangeDeg:       r.f32(),
			ServoReversed:       r.flag(),
			HardwareWatchdogMS:  r.u16(),
			StatusIntervalMS:    r.u16(),
			OdometryIntervalMS:  r.u16(),
		}
	case TypeCommand:
		p.Command = Command{SpeedMPS: r.f32(), SteeringAngleRad: r.f32()}
	case TypePing:
		p.Ping = Ping{HostTimeUS: r.u64()}
	case TypePong:
		p.Pong = Pong{HostTimeUS: r.u64(), BoardTimeUS: r.u64()}
	case TypeStatus:
		p.Status = Status{
			BoardTimeUS:   r.u64(),
			State:         State(r.u8()),
			Faults:        r.u8(),
			Duty:          r.f32(),
			ServoAngleDeg: r.f32(),
			CommandAgeMS:  r.u32(),
		}
	case TypeOdometry:
		p.Odometry = Odometry{BoardTimeUS: r.u64(), Counts: int64(r.u64())}
	}
}
