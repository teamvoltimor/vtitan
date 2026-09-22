package boardlink

import (
	"bytes"
	"errors"
	"math"
	"testing"
)

func samplePackets() []Packet {
	return []Packet{
		{
			Seq:   1,
			Type:  TypeHello,
			Hello: Hello{ProtocolVersion: Version, BootID: 0xDEADBEEF, Faults: FaultWatchdogReset},
		},
		{Seq: 2, Type: TypeConfig, Config: Config{
			CommandTimeoutMS: 500, SpeedScalePctPerMPS: 30, InvertDrive: true, LinkageRatio: 0.63,
			ServoMaxAngleDeg: 135, SteeringOffsetDeg: -1.5, ServoMinPulseUS: 500, ServoMaxPulseUS: 2500,
			ServoCenterPulseUS: 1500, ServoRangeDeg: 270, ServoReversed: true,
			HardwareWatchdogMS: 250, StatusIntervalMS: 100, OdometryIntervalMS: 20,
		}},
		{Seq: 3, Type: TypeCommand, Command: Command{SpeedMPS: 0.5, SteeringAngleRad: -0.3}},
		{Seq: 4, Type: TypePing, Ping: Ping{HostTimeUS: 1 << 40}},
		{Seq: 5, Type: TypePong, Pong: Pong{HostTimeUS: 1 << 40, BoardTimeUS: 123456789}},
		{Seq: 6, Type: TypeStatus, Status: Status{
			BoardTimeUS:   42,
			State:         StateRunning,
			Faults:        FaultActuator,
			Duty:          -0.25,
			ServoAngleDeg: 12.5,
			CommandAgeMS:  17,
		}},
		{Seq: 0xFFFF, Type: TypeOdometry, Odometry: Odometry{BoardTimeUS: 99, Counts: -123456}},
		{Seq: 8, Type: TypeButton, Button: Button{BoardTimeUS: 7, Pressed: true}},
	}
}

// feedAll runs every byte through d and returns the packets and errors it
// produced, in order.
func feedAll(d *Decoder, stream []byte) ([]Packet, []error) {
	var got []Packet
	var errs []error
	for _, b := range stream {
		var p Packet
		ok, err := d.Feed(b, &p)
		if err != nil {
			errs = append(errs, err)
		}
		if ok {
			got = append(got, p)
		}
	}
	return got, errs
}

func TestRoundTripEveryType(t *testing.T) {
	t.Parallel()

	var stream []byte
	want := samplePackets()
	for i := range want {
		var err error
		stream, err = Append(stream, &want[i])
		if err != nil {
			t.Fatalf("Append(%v): %v", want[i].Type, err)
		}
	}

	got, errs := feedAll(&Decoder{}, stream)
	if len(errs) != 0 {
		t.Fatalf("decode errors: %v", errs)
	}
	if len(got) != len(want) {
		t.Fatalf("decoded %d packets, want %d", len(got), len(want))
	}
	for i := range want {
		if got[i] != want[i] {
			t.Errorf("packet %d:\n got %+v\nwant %+v", i, got[i], want[i])
		}
	}
}

func TestEncodedFrameHasNoZeroBeforeDelimiter(t *testing.T) {
	t.Parallel()

	pkts := samplePackets()
	for i := range pkts {
		p := &pkts[i]
		frame, err := Append(nil, p)
		if err != nil {
			t.Fatal(err)
		}
		if z := bytes.IndexByte(frame, 0); z != len(frame)-1 {
			t.Errorf("%v: zero at %d of %d, want only the final delimiter", p.Type, z, len(frame))
		}
		if len(frame) > MaxEncodedLen {
			t.Errorf("%v: frame is %d bytes, over MaxEncodedLen %d", p.Type, len(frame), MaxEncodedLen)
		}
	}
}

// A receiver that joins mid-frame, or loses bytes, must drop at most the
// damaged frame and decode the next one intact.
func TestResynchronisesAfterGarbageAndCorruption(t *testing.T) {
	t.Parallel()

	pkts := samplePackets()
	first, _ := Append(nil, &pkts[2])
	second, _ := Append(nil, &pkts[5])

	corrupt := append([]byte(nil), first...)
	corrupt[3] ^= 0x40

	stream := make([]byte, 0, 4+len(corrupt)+len(second))
	stream = append(stream, 0x13, 0x37, 0xAA, 0) // tail of a frame we joined late
	stream = append(stream, corrupt...)
	stream = append(stream, second...)

	got, errs := feedAll(&Decoder{}, stream)
	if len(got) != 1 || got[0] != pkts[5] {
		t.Fatalf("got %+v, want only the intact status packet", got)
	}
	if len(errs) != 2 {
		t.Fatalf("errors %v, want one for the joined tail and one for the corrupted frame", errs)
	}
}

func TestRejects(t *testing.T) {
	t.Parallel()

	good, _ := Append(nil, &Packet{Type: TypeCommand, Command: Command{SpeedMPS: 1}})

	// Re-encode a raw frame with a valid CRC, so each case reaches the check
	// it is about rather than failing on the CRC first.
	build := func(raw []byte) []byte {
		w := writer{b: append([]byte(nil), raw...)}
		w.u16(crc16(raw))
		return append(appendCOBS(nil, w.b), 0)
	}

	cases := map[string]struct {
		stream []byte
		want   error
	}{
		"bad crc": {stream: func() []byte {
			b := append([]byte(nil), good...)
			b[len(b)-2] ^= 0x01
			return b
		}(), want: ErrCRC},
		"future version": {
			stream: build([]byte{Version + 1, uint8(TypePing), 0, 0, 1, 2, 3, 4, 5, 6, 7, 8}),
			want:   ErrVersion,
		},
		"unknown type": {stream: build([]byte{Version, 0x7F, 0, 0}), want: ErrUnknownType},
		"short body":   {stream: build([]byte{Version, uint8(TypePing), 0, 0, 1, 2, 3}), want: ErrLength},
		"overflow":     {stream: append(bytes.Repeat([]byte{1}, MaxEncodedLen+5), 0), want: ErrOverflow},
	}
	for name, tc := range cases {
		_, errs := feedAll(&Decoder{}, tc.stream)
		if len(errs) != 1 || !errors.Is(errs[0], tc.want) {
			t.Errorf("%s: errors %v, want exactly [%v]", name, errs, tc.want)
		}
	}
}

func TestAppendRejectsUnknownType(t *testing.T) {
	t.Parallel()

	if _, err := Append(nil, &Packet{Type: 0x7F}); !errors.Is(err, ErrUnknownType) {
		t.Fatalf("Append(unknown) error = %v, want ErrUnknownType", err)
	}
}

// Non-finite floats are data here, not errors: the protocol carries what the
// host sent, bit for bit, and the board's finite-command rule rejects it.
// Normalising NaN in transit would hide a host bug behind a plausible value.
func TestCarriesNonFiniteFloatsBitForBit(t *testing.T) {
	t.Parallel()

	in := Packet{Type: TypeCommand, Command: Command{
		SpeedMPS: float32(math.Inf(1)), SteeringAngleRad: float32(math.NaN()),
	}}
	frame, _ := Append(nil, &in)
	got, _ := feedAll(&Decoder{}, frame)
	if len(got) != 1 || !math.IsInf(float64(got[0].Command.SpeedMPS), 1) ||
		!math.IsNaN(float64(got[0].Command.SteeringAngleRad)) {
		t.Fatalf("got %+v, want +Inf speed and NaN steering carried through", got)
	}
}

// The known answer for CRC-16/CCITT-FALSE over "123456789" is 0x29B1.
func TestCRC16KnownAnswer(t *testing.T) {
	t.Parallel()

	if got := crc16([]byte("123456789")); got != 0x29B1 {
		t.Fatalf("crc16 = 0x%04X, want 0x29B1", got)
	}
}

func TestCOBSRoundTripEdgeCases(t *testing.T) {
	t.Parallel()

	cases := [][]byte{
		{},
		{0},
		{0, 0},
		{1, 0, 2},
		bytes.Repeat([]byte{7}, 254),
		bytes.Repeat([]byte{7}, 255),
		append(bytes.Repeat([]byte{7}, 254), 0, 9),
	}
	for _, src := range cases {
		enc := appendCOBS(nil, src)
		if bytes.IndexByte(enc, 0) != -1 {
			t.Errorf("COBS(%v) contains a zero: %v", src, enc)
		}
		dst := make([]byte, len(src)+8)
		n, ok := decodeCOBS(dst, enc)
		if !ok || !bytes.Equal(dst[:n], src) {
			t.Errorf("COBS round trip of %d bytes: ok=%v got %v", len(src), ok, dst[:n])
		}
	}
}

func FuzzDecoderNeverPanics(f *testing.F) {
	pkts := samplePackets()
	for i := range pkts {
		frame, _ := Append(nil, &pkts[i])
		f.Add(frame)
	}
	f.Fuzz(func(t *testing.T, stream []byte) {
		d := &Decoder{}
		var p Packet
		for _, b := range stream {
			_, _ = d.Feed(b, &p)
		}
	})
}
