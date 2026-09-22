package boardloop

import (
	"context"
	"errors"
	"fmt"
	"math"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/actuation"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/servo"
)

// Link is the byte stream to the host (USB CDC or a UART on the Pico).
type Link interface {
	// Read copies bytes already received into p and returns at once,
	// with 0 when there are none. It must not block.
	Read(p []byte) (int, error)
	// Write sends one whole encoded frame.
	Write(p []byte) (int, error)
}

// Drive is the drive motor; *hbridge.Controller satisfies it. It must be
// built with invert false: the Loop applies Config.InvertDrive itself (see
// the package doc).
type Drive interface {
	// Connect brings the drive up at zero duty. The Loop calls it once,
	// on the first valid Config, and again on a later Config only if it
	// failed.
	Connect(ctx context.Context) error
	// SetSpeed drives at a signed duty fraction in [-1, 1].
	SetSpeed(ctx context.Context, normalized float64) error
}

// Servo is the steering servo's PWM output.
type Servo interface {
	// SetPulseUS sets the pulse width, in microseconds, the servo is sent
	// every frame. The Loop dedupes with servo.NeedsWrite, so it is called
	// only when the pulse changes by at least servo.EpsilonUS.
	SetPulseUS(pulseUS float64) error
}

// Encoder is the wheel encoder.
type Encoder interface {
	// Counts is the signed quadrature count since boot.
	Counts() int64
}

// Hardware is what the Loop drives. Link, Drive and Servo are required;
// Encoder may be nil, and then no Odometry is sent.
type Hardware struct {
	Link    Link
	Drive   Drive
	Servo   Servo
	Encoder Encoder
}

// Options are the boot facts the firmware knows and the Loop reports.
type Options struct {
	// BootID is random per boot (boardlink.Hello.BootID).
	BootID uint32
	// BootFaults are fault bits that hold for the whole boot, reported in
	// every Hello and Status: boardlink.FaultWatchdogReset when the boot
	// followed a hardware watchdog reset.
	BootFaults uint8
}

// Counters count what the Loop dropped or failed to do, for the firmware's
// diagnostics and for tests. They wrap at 2^32.
type Counters struct {
	FramesReceived   uint32
	FramesDropped    uint32
	FramesUnexpected uint32
	FramesSent       uint32
	LinkReadErrors   uint32
	LinkWriteErrors  uint32
	ConfigsApplied   uint32
	ConfigsRejected  uint32
	CommandsApplied  uint32
	CommandsIgnored  uint32
	CommandsRejected uint32
	WatchdogStops    uint32
}

// Loop is the board's control loop. Build it with New and call Step in a
// tight loop. Not safe for concurrent use.
type Loop struct {
	hw   Hardware
	opts Options

	dec   boardlink.Decoder
	rx    boardlink.Packet
	tx    boardlink.Packet
	rxBuf [readChunk]byte
	txBuf [boardlink.MaxEncodedLen]byte
	seq   uint16

	cfg            session
	configured     bool
	driveConnected bool

	watchdog      actuation.Watchdog
	lastAcceptAt  time.Duration
	duty          float64
	servoDeg      float64
	pulseUS       float64
	hasPulse      bool
	actuatorFault bool
	connectFault  bool
	rejected      bool

	nextHello    time.Duration
	nextStatus   time.Duration
	nextOdometry time.Duration

	counters Counters
}

// HelloInterval is how often an unconfigured board repeats Hello.
const HelloInterval = 250 * time.Millisecond

const (
	// readChunk is how many bytes one Link.Read may return: a little over
	// one of the largest frames.
	readChunk = 64
	// maxReadsPerStep bounds the bytes one Step consumes, so a flooded
	// link cannot starve the watchdog check behind it.
	maxReadsPerStep = 8
)

// ErrMissingHardware is returned by New when Link, Drive or Servo is nil.
var ErrMissingHardware = errors.New("boardloop: Link, Drive and Servo are required")

// New builds an unconfigured Loop. It touches no hardware: until a valid
// Config arrives the drive is not connected and the servo is not written.
func New(hw Hardware, opts Options) (*Loop, error) {
	if hw.Link == nil || hw.Drive == nil || hw.Servo == nil {
		return nil, ErrMissingHardware
	}
	return &Loop{hw: hw, opts: opts}, nil
}

// Step runs one iteration at now, the time since boot: it reads and applies
// what the link has received, checks the command watchdog, and sends what
// is due. now must not go backwards.
func (l *Loop) Step(now time.Duration) {
	l.receive(now)
	if l.configured {
		l.checkWatchdog(now)
	}
	l.sendDue(now)
}

// Config returns the Config in force and whether the board is configured.
// The firmware reads HardwareWatchdogMS from it.
func (l *Loop) Config() (boardlink.Config, bool) {
	return l.cfg.wire, l.configured
}

// Counters returns a copy of the diagnostic counters.
func (l *Loop) Counters() Counters {
	return l.counters
}

// receive drains up to maxReadsPerStep reads from the link through the
// decoder and handles every complete frame.
func (l *Loop) receive(now time.Duration) {
	for range maxReadsPerStep {
		n, err := l.hw.Link.Read(l.rxBuf[:])
		if err != nil {
			l.counters.LinkReadErrors++
		}
		n = max(0, min(n, len(l.rxBuf)))
		for _, b := range l.rxBuf[:n] {
			ok, ferr := l.dec.Feed(b, &l.rx)
			switch {
			case ferr != nil:
				l.counters.FramesDropped++
			case ok:
				l.counters.FramesReceived++
				l.handle(now)
			}
		}
		if err != nil || n < len(l.rxBuf) {
			return
		}
	}
}

// handle acts on the frame just decoded into l.rx.
func (l *Loop) handle(now time.Duration) {
	switch l.rx.Type {
	case boardlink.TypeConfig:
		l.applyConfig(now, l.rx.Config)
	case boardlink.TypeCommand:
		l.applyCommand(now, l.rx.Command)
	case boardlink.TypePing:
		l.tx.Type = boardlink.TypePong
		l.tx.Pong = boardlink.Pong{HostTimeUS: l.rx.Ping.HostTimeUS, BoardTimeUS: micros(now)}
		l.send()
	default:
		l.counters.FramesUnexpected++
	}
}

// applyConfig applies c if it is valid, per the package doc's Session.
func (l *Loop) applyConfig(now time.Duration, c boardlink.Config) {
	cfg, err := newSession(c)
	if err != nil {
		l.counters.ConfigsRejected++
		l.unconfigure(now)
		return
	}
	if !l.driveConnected {
		if cerr := l.hw.Drive.Connect(context.Background()); cerr != nil {
			l.counters.ConfigsRejected++
			l.connectFault = true
			l.unconfigure(now)
			return
		}
		l.driveConnected = true
		l.connectFault = false
	}

	l.cfg = cfg
	l.configured = true
	l.counters.ConfigsApplied++
	l.watchdog = actuation.NewWatchdog(epoch(now))
	l.lastAcceptAt = now
	// The new calibration may map center to a different pulse: never skip
	// the first write under it.
	l.hasPulse = false
	l.rejected = false
	l.safeStop()

	l.sendStatus(now)
	l.nextStatus = now + cfg.statusEvery
	l.nextOdometry = now
}

// unconfigure safety-stops a configured board with the calibration it has
// and drops it back to Hello. On an unconfigured board it only restarts the
// Hello cadence.
func (l *Loop) unconfigure(now time.Duration) {
	if l.configured {
		l.safeStop()
		l.watchdog.Stop()
		l.configured = false
	}
	l.nextHello = now
}

// applyCommand is internal/node/motor's applyCommand: reject a non-finite
// command whole without refreshing the watchdog, else steer, then drive.
func (l *Loop) applyCommand(now time.Duration, c boardlink.Command) {
	if !l.configured {
		l.counters.CommandsIgnored++
		return
	}
	if !actuation.FiniteCommand(float64(c.SpeedMPS), float64(c.SteeringAngleRad)) {
		l.counters.CommandsRejected++
		l.rejected = true
		return
	}

	l.counters.CommandsApplied++
	l.watchdog.Accept(epoch(now))
	l.lastAcceptAt = now
	servoDeg, _ := actuation.SteeringToServoDeg(c.SteeringAngleRad, l.cfg.steering)
	steerErr := l.steer(servoDeg)
	driveErr := l.drive(actuation.SpeedToNormalized(c.SpeedMPS, l.cfg.speedScale))
	l.actuatorFault = steerErr != nil || driveErr != nil
}

// checkWatchdog stops the drive and centers the steering, once per
// staleness episode, and reports it at once.
func (l *Loop) checkWatchdog(now time.Duration) {
	if _, expired := l.watchdog.Expire(epoch(now), l.cfg.commandTimeout); !expired {
		return
	}
	l.counters.WatchdogStops++
	l.safeStop()
	l.sendStatus(now)
	l.nextStatus = now + l.cfg.statusEvery
}

// safeStop zeroes the drive and centers the steering, drive first: of the
// two, a moving wheel is the hazard.
func (l *Loop) safeStop() {
	driveErr := l.drive(0)
	steerErr := l.steer(actuation.SteeringCenterDeg)
	l.actuatorFault = steerErr != nil || driveErr != nil
}

// steer writes the pulse for servoDeg if it moved by servo.EpsilonUS. A
// failed write forgets the last pulse, so the next one is retried even if
// it is the same.
func (l *Loop) steer(servoDeg float64) error {
	l.servoDeg = servoDeg
	pulse := servo.PulseUS(l.cfg.pulse, servoDeg)
	if !servo.NeedsWrite(l.pulseUS, l.hasPulse, pulse) {
		return nil
	}
	if err := l.hw.Servo.SetPulseUS(pulse); err != nil {
		l.hasPulse = false
		return fmt.Errorf("boardloop: servo: %w", err)
	}
	l.pulseUS = pulse
	l.hasPulse = true
	return nil
}

// drive records duty as commanded and writes it, negated when the Config
// says the drive is inverted.
func (l *Loop) drive(duty float64) error {
	l.duty = duty
	if l.cfg.wire.InvertDrive {
		duty = -duty
	}
	if err := l.hw.Drive.SetSpeed(context.Background(), duty); err != nil {
		return fmt.Errorf("boardloop: drive: %w", err)
	}
	return nil
}

// sendDue sends Hello while unconfigured, and Status and Odometry once
// configured, each when its interval has elapsed. The next deadline is
// counted from now, not from the missed one, so a stalled loop does not
// burst on recovery.
func (l *Loop) sendDue(now time.Duration) {
	if !l.configured {
		if now >= l.nextHello {
			l.sendHello()
			l.nextHello = now + HelloInterval
		}
		return
	}
	if now >= l.nextStatus {
		l.sendStatus(now)
		l.nextStatus = now + l.cfg.statusEvery
	}
	if l.hw.Encoder != nil && l.cfg.odometryEvery > 0 && now >= l.nextOdometry {
		l.tx.Type = boardlink.TypeOdometry
		l.tx.Odometry = boardlink.Odometry{BoardTimeUS: micros(now), Counts: l.hw.Encoder.Counts()}
		l.send()
		l.nextOdometry = now + l.cfg.odometryEvery
	}
}

func (l *Loop) sendHello() {
	faults := l.opts.BootFaults
	if l.connectFault {
		faults |= boardlink.FaultActuator
	}
	l.tx.Type = boardlink.TypeHello
	l.tx.Hello = boardlink.Hello{ProtocolVersion: boardlink.Version, BootID: l.opts.BootID, Faults: faults}
	l.send()
}

// sendStatus reports the state now and clears FaultRejectedCommand.
func (l *Loop) sendStatus(now time.Duration) {
	state := boardlink.StateIdle
	faults := l.opts.BootFaults
	switch {
	case !l.configured:
		state = boardlink.StateUnconfigured
	case l.actuatorFault:
		state = boardlink.StateFault
	case l.duty != 0:
		state = boardlink.StateRunning
	}
	if l.actuatorFault {
		faults |= boardlink.FaultActuator
	}
	if l.rejected {
		faults |= boardlink.FaultRejectedCommand
		l.rejected = false
	}
	l.tx.Type = boardlink.TypeStatus
	l.tx.Status = boardlink.Status{
		BoardTimeUS:   micros(now),
		State:         state,
		Faults:        faults,
		Duty:          float32(l.duty),
		ServoAngleDeg: float32(l.servoDeg),
		CommandAgeMS:  millis(now - l.lastAcceptAt),
	}
	l.send()
}

// send encodes l.tx with the next sequence number into the fixed transmit
// buffer and writes it. A frame that fails to encode or write is counted
// and dropped; the sequence number is spent either way, so the host sees
// the gap.
func (l *Loop) send() {
	l.tx.Seq = l.seq
	l.seq++
	frame, err := boardlink.Append(l.txBuf[:0], &l.tx)
	if err != nil {
		l.counters.LinkWriteErrors++
		return
	}
	if n, werr := l.hw.Link.Write(frame); werr != nil || n != len(frame) {
		l.counters.LinkWriteErrors++
		return
	}
	l.counters.FramesSent++
}

// epoch turns a time since boot into the time.Time actuation.Watchdog
// takes. Only differences between two such values are ever used.
func epoch(sinceBoot time.Duration) time.Time {
	return time.Time{}.Add(sinceBoot)
}

func micros(d time.Duration) uint64 {
	return uint64(max(0, d.Microseconds()))
}

func millis(d time.Duration) uint32 {
	return uint32(min(max(0, d.Milliseconds()), math.MaxUint32))
}
