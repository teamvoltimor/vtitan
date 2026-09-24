package boardsim

import (
	"bytes"
	"context"
	"math/bits"
	"math/rand/v2"
	"net"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
)

// validConfig passes boardloop.ValidateConfig: the calibration of
// boardloop's own tests.
var validConfig = boardlink.Config{
	CommandTimeoutMS:    500,
	SpeedScalePctPerMPS: 30,
	LinkageRatio:        0.5,
	ServoMaxAngleDeg:    135,
	ServoMinPulseUS:     500,
	ServoMaxPulseUS:     2500,
	ServoCenterPulseUS:  1500,
	ServoRangeDeg:       270,
	HardwareWatchdogMS:  250,
	StatusIntervalMS:    100,
	OdometryIntervalMS:  20,
}

// startBoard runs a Board over a net.Pipe and returns the host end.
func startBoard(t *testing.T, opts Options) (*Board, net.Conn) {
	t.Helper()

	hostEnd, boardEnd := net.Pipe()
	b, err := New(boardEnd, opts)
	if err != nil {
		t.Fatalf("New: %v", err)
	}
	ctx, cancel := context.WithCancel(context.Background())
	done := make(chan struct{})
	go func() {
		defer close(done)
		_ = b.Run(ctx)
	}()
	t.Cleanup(func() {
		cancel()
		_ = hostEnd.Close()
		_ = boardEnd.Close()
		<-done
	})
	return b, hostEnd
}

// readUntil reads host until a packet of type want arrives.
func readUntil(t *testing.T, host net.Conn, want boardlink.Type) {
	t.Helper()

	var (
		dec boardlink.Decoder
		pkt boardlink.Packet
	)
	buf := make([]byte, boardlink.MaxEncodedLen)
	_ = host.SetReadDeadline(time.Now().Add(2 * time.Second))
	for {
		n, err := host.Read(buf)
		if err != nil {
			t.Fatalf("waiting for %s: %v", want, err)
		}
		for _, c := range buf[:n] {
			if ok, _ := dec.Feed(c, &pkt); ok && pkt.Type == want {
				return
			}
		}
	}
}

// Each direction is delayed by its own latency: the first Hello reaches
// the host no sooner than ToHost.Latency after boot, and a Config takes
// effect no sooner than ToBoard.Latency after the host sends it.
func TestLink_LatencyDelaysEachDirection(t *testing.T) {
	t.Parallel()

	const toHost, toBoard = 60 * time.Millisecond, 90 * time.Millisecond
	start := time.Now()
	b, host := startBoard(t, Options{Link: LinkConfig{
		ToHost:  Direction{Latency: toHost},
		ToBoard: Direction{Latency: toBoard},
	}})

	readUntil(t, host, boardlink.TypeHello)
	if got := time.Since(start); got < toHost {
		t.Errorf("first Hello after %v, want at least %v", got, toHost)
	}

	frame, err := boardlink.Append(nil, &boardlink.Packet{Type: boardlink.TypeConfig, Config: validConfig})
	if err != nil {
		t.Fatalf("encoding Config: %v", err)
	}
	// Drain the board's output meanwhile, or its writes block the pipe.
	go func() {
		buf := make([]byte, 4096)
		for {
			if _, readErr := host.Read(buf); readErr != nil {
				return
			}
		}
	}()
	sent := time.Now()
	if _, err = host.Write(frame); err != nil {
		t.Fatalf("writing Config: %v", err)
	}
	for {
		if _, ok := b.Configured(); ok {
			break
		}
		if time.Since(sent) > 2*time.Second {
			t.Fatal("board never configured")
		}
		time.Sleep(time.Millisecond)
	}
	if got := time.Since(sent); got < toBoard {
		t.Errorf("configured %v after the Config was sent, want at least %v", got, toBoard)
	}
}

// Jitter delays chunks unevenly but never reorders them: the link is a
// byte stream.
func TestLink_JitterKeepsOrder(t *testing.T) {
	t.Parallel()

	hostEnd, boardEnd := net.Pipe()
	t.Cleanup(func() { _ = hostEnd.Close(); _ = boardEnd.Close() })
	l := newLink(boardEnd, LinkConfig{
		ToBoard: Direction{Jitter: 20 * time.Millisecond},
		ToHost:  Direction{Jitter: 20 * time.Millisecond},
		Seed:    3,
	})

	const n = 50
	go func() {
		for i := range n {
			_, _ = hostEnd.Write([]byte{byte(i)})
		}
	}()
	var got []byte
	buf := make([]byte, 16)
	deadline := time.Now().Add(2 * time.Second)
	for len(got) < n && time.Now().Before(deadline) {
		k, _ := l.Read(buf)
		got = append(got, buf[:k]...)
		time.Sleep(100 * time.Microsecond)
	}
	for i, c := range got {
		if int(c) != i {
			t.Fatalf("to board: byte %d = %d, want %d (got %v)", i, c, i, got)
		}
	}

	for i := range n {
		if _, err := l.Write([]byte{byte(i)}); err != nil {
			t.Fatalf("Write %d: %v", i, err)
		}
	}
	back := make([]byte, n)
	_ = hostEnd.SetReadDeadline(time.Now().Add(2 * time.Second))
	for read := 0; read < n; {
		k, err := hostEnd.Read(back[read:])
		if err != nil {
			t.Fatalf("to host: reading: %v", err)
		}
		read += k
	}
	for i, c := range back {
		if int(c) != i {
			t.Fatalf("to host: byte %d = %d, want %d", i, c, i)
		}
	}
}

// A rate of 0 leaves data alone, a rate of 1 flips exactly one bit in
// every byte, and the same seed corrupts the same way.
func TestCorrupt(t *testing.T) {
	t.Parallel()

	orig := bytes.Repeat([]byte{0x00, 0xFF, 0x5A}, 100)

	if got := corrupt(bytes.Clone(orig), 0, rand.New(rand.NewPCG(1, 1))); !bytes.Equal(got, orig) {
		t.Error("rate 0 changed the data")
	}

	got := corrupt(bytes.Clone(orig), 1, rand.New(rand.NewPCG(1, 1)))
	for i := range orig {
		if d := bits.OnesCount8(got[i] ^ orig[i]); d != 1 {
			t.Fatalf("rate 1: byte %d differs in %d bits, want 1", i, d)
		}
	}

	a := corrupt(bytes.Clone(orig), 0.1, rand.New(rand.NewPCG(9, 1)))
	b := corrupt(bytes.Clone(orig), 0.1, rand.New(rand.NewPCG(9, 1)))
	if !bytes.Equal(a, b) {
		t.Error("the same seed corrupted differently")
	}
}

func TestLinkConfig_Validate(t *testing.T) {
	t.Parallel()

	cases := map[string]struct {
		cfg   LinkConfig
		valid bool
	}{
		"ideal": {LinkConfig{}, true},
		"typical": {
			LinkConfig{ToHost: Direction{Latency: time.Millisecond, Jitter: time.Millisecond, CorruptRate: 0.01}},
			true,
		},
		"negative latency": {LinkConfig{ToBoard: Direction{Latency: -time.Millisecond}}, false},
		"negative jitter":  {LinkConfig{ToHost: Direction{Jitter: -time.Millisecond}}, false},
		"rate above one":   {LinkConfig{ToHost: Direction{CorruptRate: 1.5}}, false},
		"negative rate":    {LinkConfig{ToBoard: Direction{CorruptRate: -0.1}}, false},
	}
	for name, tc := range cases {
		if err := tc.cfg.Validate(); (err == nil) != tc.valid {
			t.Errorf("%s: Validate = %v, want valid %v", name, err, tc.valid)
		}
	}
}
