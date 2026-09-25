package boardsim

import (
	"bytes"
	"context"
	"math"
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

	if got := bytes.Clone(orig); corrupt(got, 0, rand.New(rand.NewPCG(1, 1))) != 0 || !bytes.Equal(got, orig) {
		t.Error("rate 0 changed the data")
	}

	got := bytes.Clone(orig)
	if n := corrupt(got, 1, rand.New(rand.NewPCG(1, 1))); n != len(orig) {
		t.Errorf("rate 1 reported %d flipped bytes, want %d", n, len(orig))
	}
	for i := range orig {
		if d := bits.OnesCount8(got[i] ^ orig[i]); d != 1 {
			t.Fatalf("rate 1: byte %d differs in %d bits, want 1", i, d)
		}
	}

	a, b := bytes.Clone(orig), bytes.Clone(orig)
	corrupt(a, 0.1, rand.New(rand.NewPCG(9, 1)))
	corrupt(b, 0.1, rand.New(rand.NewPCG(9, 1)))
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
		"loss of one":      {LinkConfig{ToBoard: Direction{LossRate: 1, LossBurst: 2}}, false},
		"burst below one":  {LinkConfig{ToHost: Direction{LossRate: 0.1, LossBurst: 0.5}}, false},
		"negative stall":   {LinkConfig{ToHost: Direction{StallRate: -1}}, false},
		"bursty loss":      {LinkConfig{ToHost: Direction{LossRate: 0.1, LossBurst: 3}}, true},
	}
	for name, tc := range cases {
		if err := tc.cfg.Validate(); (err == nil) != tc.valid {
			t.Errorf("%s: Validate = %v, want valid %v", name, err, tc.valid)
		}
	}
}

// The Gilbert-Elliott chain loses LossRate of chunks in the long run, in
// bursts of LossBurst chunks on average.
func TestPath_LossMatchesRateAndBurst(t *testing.T) {
	t.Parallel()

	const rate, burst, n = 0.2, 4.0, 200_000
	p := path{cfg: Direction{LossRate: rate, LossBurst: burst}, rand: rand.New(rand.NewPCG(5, 1))}
	now := time.Unix(0, 0)
	lost, bursts := 0, 0
	prevLost := false
	for range n {
		_, _, l := p.admit([]byte{0}, now)
		if l {
			lost++
			if !prevLost {
				bursts++
			}
		}
		prevLost = l
	}
	if got := float64(lost) / n; math.Abs(got-rate) > 0.01 {
		t.Errorf("loss fraction = %.4f, want %.2f +- 0.01", got, rate)
	}
	if got := float64(lost) / float64(bursts); math.Abs(got-burst) > 0.2 {
		t.Errorf("mean burst = %.3f chunks, want %.1f +- 0.2", got, burst)
	}
	if p.stats.ChunksLost != uint64(lost) || p.stats.Chunks != n {
		t.Errorf("stats = %+v, want %d lost of %d", p.stats, lost, n)
	}
}

// Random stalls start at StallRate per second, and a chunk sent during one
// is held until it ends.
func TestPath_RandomStallsHoldDelivery(t *testing.T) {
	t.Parallel()

	const rate, dur = 10.0, 50 * time.Millisecond
	p := path{cfg: Direction{StallRate: rate, StallDuration: dur}, rand: rand.New(rand.NewPCG(8, 1))}
	start := time.Unix(0, 0)
	held := 0
	for ms := range 60_000 {
		now := start.Add(time.Duration(ms) * time.Millisecond)
		_, due, _ := p.admit([]byte{0}, now)
		if due.After(now) {
			held++
			if !due.Equal(p.holdUntil) {
				t.Fatalf("at %v a held chunk is due %v, want the stall's end %v", now, due, p.holdUntil)
			}
		}
	}
	// 60 s at 10/s with 50 ms of dead time each: about 60/(0.1+0.05) = 400.
	if s := p.stats.Stalls; s < 330 || s > 470 {
		t.Errorf("stalls in 60 s = %d, want about 400", s)
	}
	if held == 0 {
		t.Error("no chunk was ever held")
	}
}

// Reboot starts a new boot: a Hello with the new BootID and the reset
// faults, from a board that is unconfigured again.
func TestBoard_RebootAnnouncesANewBoot(t *testing.T) {
	t.Parallel()

	b, host := startBoard(t, Options{BootID: 1})
	readUntil(t, host, boardlink.TypeHello)

	if err := b.Reboot(2, boardlink.FaultWatchdogReset); err != nil {
		t.Fatalf("Reboot: %v", err)
	}
	var (
		dec boardlink.Decoder
		pkt boardlink.Packet
	)
	buf := make([]byte, boardlink.MaxEncodedLen)
	_ = host.SetReadDeadline(time.Now().Add(2 * time.Second))
	for {
		n, err := host.Read(buf)
		if err != nil {
			t.Fatalf("waiting for the new Hello: %v", err)
		}
		for _, c := range buf[:n] {
			if ok, _ := dec.Feed(c, &pkt); ok && pkt.Type == boardlink.TypeHello && pkt.Hello.BootID == 2 {
				if pkt.Hello.Faults&boardlink.FaultWatchdogReset == 0 {
					t.Errorf("Hello faults = %v, want FaultWatchdogReset", pkt.Hello.Faults)
				}
				if _, configured := b.Configured(); configured {
					t.Error("board configured right after a reboot")
				}
				return
			}
		}
	}
}
