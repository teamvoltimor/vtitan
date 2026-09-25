package picolink

import (
	"context"
	"errors"
	"fmt"
	"io"
	"log/slog"
	"sync"
	"sync/atomic"
	"time"

	nodebutton "github.com/teamvoltimor/vtitan/src/go/internal/node/button"
	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
	uiv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/ui/v1"
	driverbutton "github.com/teamvoltimor/vtitan/src/go/pkg/driver/button"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardlink"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/boardloop"
	"github.com/teamvoltimor/vtitan/src/go/pkg/portable/quadrature"
)

// StatusPublisher is the transport call Session makes for Status;
// *nats.Publisher[*actuationv1.MotorStatus] satisfies it.
type StatusPublisher interface {
	Publish(msg *actuationv1.MotorStatus) error
}

// JointStatesPublisher is the transport call Session makes for Odometry;
// *nats.Publisher[*actuationv1.JointStates] satisfies it.
type JointStatesPublisher interface {
	Publish(msg *actuationv1.JointStates) error
}

// CommandReader is the transport call Session makes on the way in;
// *nats.Subscriber[*actuationv1.AckermannCmd] satisfies it.
type CommandReader interface {
	Read(ctx context.Context) (*actuationv1.AckermannCmd, error)
}

// ButtonPublisher is the transport call Session makes for an evaluated
// button event; *nats.Publisher[*uiv1.ButtonEvent] satisfies it.
type ButtonPublisher interface {
	Publish(msg *uiv1.ButtonEvent) error
}

// ButtonHoldPublisher is the transport call Session makes for the
// continuous hold-progress signal; *nats.Publisher[*uiv1.ButtonHold]
// satisfies it.
type ButtonHoldPublisher interface {
	Publish(msg *uiv1.ButtonHold) error
}

// SessionConfig is what a Session needs besides its transport.
type SessionConfig struct {
	// Board is the Config sent on every Hello (see BoardConfig).
	Board boardlink.Config
	// Encoder converts Odometry into JointStates; nil skips odometry, as the
	// Zero does without an encoder profile.
	Encoder *EncoderParams
	// PingInterval is how often a Ping goes out; DefaultPingInterval when
	// not positive.
	PingInterval time.Duration
	// LinkTimeout is the silence after which the link is reported lost;
	// DefaultLinkTimeout when not positive.
	LinkTimeout time.Duration
	// Button is the host-side button evaluator's tuning; nil publishes no
	// ButtonEvent, for a board with no button.
	Button *ButtonParams
	// Lease sizes every command's lease; the zero value sends none.
	Lease LeasePolicy
}

// ClockSync is the latest clock estimate from a Ping/Pong round trip.
//
// Host times are microseconds on the monotonic clock since Epoch, so a wall
// clock step (NTP) cannot corrupt the estimate; HostTime converts back to a
// time.Time. The residual error of OffsetUS is bounded by RTT/2 (the Pong
// may have been stamped anywhere inside the round trip), which is the
// number go-future.md section 4.6 asks to state rather than assume zero.
type ClockSync struct {
	// Epoch is the host instant host microsecond 0 refers to.
	Epoch time.Time
	// RTT is the round trip of the Ping this estimate came from.
	RTT time.Duration
	// OffsetUS is board clock minus host clock, in microseconds:
	// boardTime - (hostSend+hostRecv)/2.
	OffsetUS int64
	// At is when the Pong arrived.
	At time.Time
}

// Session runs one host side of the link: the Hello/Config handshake,
// command forwarding, Status and Odometry republishing, Ping/Pong and link
// supervision. Build it with NewSession; Run is not safe to call
// concurrently, and Clock is safe to call from any goroutine.
type Session struct {
	logger       *slog.Logger
	board        boardlink.Config
	encoder      *EncoderParams
	status       StatusPublisher
	joints       JointStatesPublisher
	pingInterval time.Duration
	linkTimeout  time.Duration
	epoch        time.Time
	// invalid is why Board fails boardloop.ValidateConfig, empty when it
	// passes. An invalid Config is never sent.
	invalid string

	// dropped counts frames the decoder rejected; written by the link
	// reader goroutine.
	dropped atomic.Uint64

	// Owned by Run's goroutine.
	bootID   uint32
	haveBoot bool
	// configPending is set when a Config is sent and cleared by the next
	// Status (the board reports at once on configuration) or Hello (it did
	// not take it).
	configPending bool
	// configuredOnce is set by the first Status after a Config. A board
	// configured by the Config Run sends first may never Hello, so the
	// session can be running without knowing the boot ID; a Hello after
	// this is a reset even then.
	configuredOnce bool
	refusals       int
	retryAfter     time.Time
	lastFaults     boardlink.Faults
	estimator      *quadrature.SpeedEstimator
	lastOdoUS      uint64
	haveOdo        bool
	lastFrameAt    time.Time
	linkLost       bool

	// buttonEval runs on the host clock, exactly as the Zero's button
	// driver does; buttonPressed is the last raw edge the board sent.
	buttonEval       *driverbutton.Evaluator
	buttonPressed    bool
	buttonPub        ButtonPublisher
	buttonThresholds driverbutton.Thresholds
	buttonHoldPub    ButtonHoldPublisher
	buttonWasPressed bool

	lease     LeasePolicy
	clockMu   sync.Mutex
	clock     ClockSync
	haveClock bool
}

// linkWriter serializes packets onto the link, owning the sequence number
// and a reused frame buffer. Only Run's goroutine writes.
type linkWriter struct {
	w       io.Writer
	buf     []byte
	seq     uint16
	started bool
}

// Link timing defaults.
const (
	// DefaultPingInterval is one Ping a second: frequent enough to track the
	// drift of a crystal-clocked board, negligible on the wire.
	DefaultPingInterval = time.Second
	// DefaultLinkTimeout is twenty Status intervals of silence.
	DefaultLinkTimeout = time.Second
	// buttonPollInterval is how often the host samples the button evaluator:
	// 20 Hz, button_node.toml's POLL_HZ, matching the Zero's button driver.
	buttonPollInterval = 50 * time.Millisecond
)

const (
	// configRetryInterval is how long, after a refused Config, the host
	// waits before answering Hello with Config again. A refusing board
	// sends Hello at once, so without it a board that cannot take its
	// Config (its drive fails to connect) and the host would trade frames as
	// fast as the link goes. It is below boardloop.HelloInterval, so the
	// board's next regular Hello is answered.
	configRetryInterval = 200 * time.Millisecond
	// linkChecksPerTimeout sets the link watchdog's poll rate relative to
	// LinkTimeout, bounding how late a loss is noticed.
	linkChecksPerTimeout = 4
	// microsPerSecond converts board microseconds into the seconds
	// quadrature.SpeedEstimator takes.
	microsPerSecond = 1e6
	// minOdometryDTS and maxOdometryDTS bound one estimator step, the same
	// bounds pkg/driver/encoder applies to its own sampling interval, so a
	// duplicated or a long-delayed sample cannot spike the speed.
	minOdometryDTS = 0.001
	maxOdometryDTS = 0.5
)

// NewSession builds a Session publishing onto status and joints. joints may
// be nil only when cfg.Encoder is nil.
func NewSession(
	logger *slog.Logger,
	cfg SessionConfig,
	status StatusPublisher,
	joints JointStatesPublisher,
	button ButtonPublisher,
	buttonHold ButtonHoldPublisher,
) (*Session, error) {
	if status == nil {
		return nil, errors.New("picolink: a MotorStatus publisher is required")
	}
	s := &Session{
		logger:       logger,
		board:        cfg.Board,
		encoder:      cfg.Encoder,
		status:       status,
		joints:       joints,
		pingInterval: cfg.PingInterval,
		lease:        cfg.Lease,
		linkTimeout:  cfg.LinkTimeout,
		epoch:        time.Now(),
	}
	if s.pingInterval <= 0 {
		s.pingInterval = DefaultPingInterval
	}
	if s.linkTimeout <= 0 {
		s.linkTimeout = DefaultLinkTimeout
	}
	if err := boardloop.ValidateConfig(cfg.Board); err != nil {
		s.invalid = err.Error()
	}
	if err := cfg.Lease.Validate(); err != nil {
		return nil, err
	}
	if cfg.Encoder != nil {
		if joints == nil {
			return nil, errors.New("picolink: an encoder is configured but no JointStates publisher was given")
		}
		est, err := quadrature.NewSpeedEstimator(cfg.Encoder.CountsPerRev)
		if err != nil {
			return nil, fmt.Errorf("picolink: %w", err)
		}
		s.estimator = est
	}
	if cfg.Button != nil {
		if button == nil {
			return nil, errors.New("picolink: button thresholds are configured but no ButtonEvent publisher was given")
		}
		if buttonHold == nil {
			return nil, errors.New("picolink: button thresholds are configured but no ButtonHold publisher was given")
		}
		s.buttonEval = driverbutton.NewEvaluator(cfg.Button.Thresholds)
		s.buttonThresholds = cfg.Button.Thresholds
		s.buttonPub = button
		s.buttonHoldPub = buttonHold
	}
	return s, nil
}

// Clock returns the latest clock estimate, and false before the first Pong.
func (s *Session) Clock() (ClockSync, bool) {
	s.clockMu.Lock()
	defer s.clockMu.Unlock()
	return s.clock, s.haveClock
}

// Dropped returns how many frames the decoder has rejected so far (CRC,
// COBS, version, length, overflow).
func (s *Session) Dropped() uint64 {
	return s.dropped.Load()
}

// Run drives the session over link until ctx is done, the link fails, or
// cmds fails. It returns nil on cancellation and an error otherwise, so a
// supervisor restarts it (and, through Run's caller, reopens the port).
//
// Every AckermannCmd is forwarded, including non-finite ones. The board
// applies pkg/portable/actuation.FiniteCommand itself, treats a rejected
// command as missing (it does not refresh the command watchdog, exactly as
// the Zero's loop), and reports it as FaultRejectedCommand. Filtering here
// would duplicate that rule and, worse, hide the fault from the one report
// that carries it. A NaN survives the wire unchanged (boardlink encodes
// float32 bits).
//
// Run sends Config first, without waiting for a Hello: a board that is
// already configured (the host restarted, or reopened the port) sends no
// Hello, and must not be left on a Config this session never checked.
// Every Hello after that is answered with Config too. A Config that fails
// boardloop.ValidateConfig is never sent: it is logged at ERROR instead,
// since the board would refuse it and drop back to unconfigured.
//
// A silent link is logged and reported, not acted on: the board's own
// command timeout stops the car when commands stop arriving, so carrying on
// is safe, and a board that comes back resumes on its next Hello.
func (s *Session) Run(ctx context.Context, link io.ReadWriter, cmds CommandReader) error {
	ctx, cancel := context.WithCancel(ctx)
	defer cancel()

	frames := make(chan boardlink.Packet)
	readErr := make(chan error, 1)
	go s.readLink(ctx, link, frames, readErr)

	cmdCh := make(chan *actuationv1.AckermannCmd)
	cmdErr := make(chan error, 1)
	go readCommands(ctx, cmds, cmdCh, cmdErr)

	ping := time.NewTicker(s.pingInterval)
	defer ping.Stop()
	watch := time.NewTicker(s.linkTimeout / linkChecksPerTimeout)
	defer watch.Stop()
	button := time.NewTicker(buttonPollInterval)
	defer button.Stop()

	w := &linkWriter{w: link}
	s.lastFrameAt = time.Now()
	if err := s.sendConfig(w); err != nil {
		return err
	}

	for {
		select {
		case <-ctx.Done():
			return nil
		case err := <-readErr:
			if ctx.Err() != nil {
				return nil
			}
			return fmt.Errorf("picolink: reading link: %w", err)
		case err := <-cmdErr:
			if ctx.Err() != nil || errors.Is(err, context.Canceled) || errors.Is(err, context.DeadlineExceeded) {
				return nil
			}
			return fmt.Errorf("picolink: reading commands: %w", err)
		case pkt := <-frames:
			if err := s.handle(w, &pkt); err != nil {
				return err
			}
		case cmd := <-cmdCh:
			pkt := boardlink.Packet{Type: boardlink.TypeCommand, Command: s.commandFor(cmd, time.Now())}
			if err := w.send(&pkt); err != nil {
				return err
			}
		case <-ping.C:
			host := s.hostMicros(time.Now())
			pingPkt := boardlink.Packet{Type: boardlink.TypePing, Ping: boardlink.Ping{HostTimeUS: host}}
			if err := w.send(&pingPkt); err != nil {
				return err
			}
		case now := <-watch.C:
			s.checkLink(now)
		case now := <-button.C:
			s.pollButton(now)
		}
	}
}

// readLink feeds link into a decoder and hands each packet to frames until
// the link errors or ctx is done. A dropped frame is counted and logged at
// DEBUG: line noise and a receiver joining mid-frame both produce them, and
// the decoder resynchronises at the next delimiter.
func (s *Session) readLink(
	ctx context.Context,
	link io.Reader,
	frames chan<- boardlink.Packet,
	errs chan<- error,
) {
	var (
		dec boardlink.Decoder
		pkt boardlink.Packet
	)
	buf := make([]byte, boardlink.ReadChunkSize)
	for {
		n, err := link.Read(buf)
		for _, b := range buf[:n] {
			ok, decErr := dec.Feed(b, &pkt)
			if decErr != nil {
				s.dropped.Add(1)
				s.logger.Debug("picolink: dropped a frame", "error", decErr)
				continue
			}
			if !ok {
				continue
			}
			select {
			case frames <- pkt:
			case <-ctx.Done():
				return
			}
		}
		if err != nil {
			errs <- err
			return
		}
		if ctx.Err() != nil {
			return
		}
	}
}

// readCommands forwards every AckermannCmd from cmds until it errors or ctx
// is done.
func readCommands(
	ctx context.Context,
	cmds CommandReader,
	out chan<- *actuationv1.AckermannCmd,
	errs chan<- error,
) {
	for {
		cmd, err := cmds.Read(ctx)
		if err != nil {
			errs <- err
			return
		}
		select {
		case out <- cmd:
		case <-ctx.Done():
			return
		}
	}
}

// handle dispatches one packet from the board.
func (s *Session) handle(w *linkWriter, pkt *boardlink.Packet) error {
	now := time.Now()
	if s.linkLost {
		s.linkLost = false
		s.logger.Info("picolink: link restored", "silence", now.Sub(s.lastFrameAt))
	}
	s.lastFrameAt = now

	switch pkt.Type {
	case boardlink.TypeHello:
		return s.onHello(w, pkt.Hello, now)
	case boardlink.TypeStatus:
		s.onStatus(pkt.Status)
	case boardlink.TypeOdometry:
		s.onOdometry(pkt.Odometry)
	case boardlink.TypePong:
		s.onPong(pkt.Pong, now)
	case boardlink.TypeButton:
		s.buttonPressed = pkt.Button.Pressed
	default:
		s.logger.Debug("picolink: ignoring a host-to-board message from the board", "type", pkt.Type.String())
	}
	return nil
}

// onHello logs the board's identity and answers with Config. A BootID
// different from the one already seen means the board reset under a running
// host: that is logged at WARN with the fault bits (FaultWatchdogReset says
// it was the hardware watchdog), and the odometry estimator is reset, since
// the board's count restarted from zero.
//
// A Hello from the same boot while a Config is pending means the board
// refused it (boardlink has no fault bit for that: a refusing board drops
// back to unconfigured and resumes Hello). The first refusal of a boot is a
// WARN, later ones DEBUG with a count, and Config is not re-sent for
// configRetryInterval. The first Hello a session sees is never counted: it
// may have crossed the Config Run sent before hearing from the board.
func (s *Session) onHello(w *linkWriter, h boardlink.Hello, now time.Time) error {
	sameBoot := s.haveBoot && h.BootID == s.bootID
	if sameBoot && s.configPending {
		s.configPending = false
		s.refusals++
		s.retryAfter = now.Add(configRetryInterval)
		if s.refusals == 1 {
			s.logger.Warn("picolink: board refused config",
				"boot_id", h.BootID, "faults", FaultString(h.Faults), "fault_bits", h.Faults)
		} else {
			s.logger.Debug("picolink: board refused config again", "boot_id", h.BootID, "refusals", s.refusals)
		}
	}

	switch {
	case !s.haveBoot && s.configuredOnce:
		// Configured before any Hello, so the previous boot ID is unknown:
		// a Hello now can only mean the board restarted.
		s.logger.Warn("picolink: board reset mid-run, reconfiguring",
			"previous_boot_id", "unknown", "boot_id", h.BootID,
			"faults", FaultString(h.Faults), "fault_bits", h.Faults)
	case !s.haveBoot:
		s.logger.Info("picolink: board hello",
			"boot_id", h.BootID, "protocol_version", h.ProtocolVersion,
			"faults", FaultString(h.Faults), "fault_bits", h.Faults)
	case h.BootID != s.bootID:
		s.logger.Warn("picolink: board reset mid-run, reconfiguring",
			"previous_boot_id", s.bootID, "boot_id", h.BootID,
			"faults", FaultString(h.Faults), "fault_bits", h.Faults)
	default:
		s.logger.Debug("picolink: repeated hello", "boot_id", h.BootID)
	}
	if !sameBoot {
		s.bootID, s.haveBoot = h.BootID, true
		s.refusals = 0
		s.retryAfter = time.Time{}
		s.resetOdometry()
		// A new boot restarted the board's clock: the old offset would put
		// every lease deadline in the wrong place until the next Pong.
		s.clockMu.Lock()
		s.haveClock = false
		s.clockMu.Unlock()
		if h.ProtocolVersion != boardlink.Version {
			s.logger.Warn("picolink: board reports a different protocol version",
				"board_version", h.ProtocolVersion, "host_version", boardlink.Version)
		}
	}
	if now.Before(s.retryAfter) {
		return nil
	}
	return s.sendConfig(w)
}

// sendConfig sends the board's Config and marks it pending, or, if the
// Config is invalid, logs why at ERROR and sends nothing.
func (s *Session) sendConfig(w *linkWriter) error {
	if s.invalid != "" {
		s.logger.Error("picolink: not sending an invalid config, the board would refuse it", "error", s.invalid)
		return nil
	}
	if err := w.send(&boardlink.Packet{Type: boardlink.TypeConfig, Config: s.board}); err != nil {
		return err
	}
	s.configPending = true
	return nil
}

// onStatus republishes Status as MotorStatus and logs the fault bits that
// MotorStatusFor does not turn into FAULT.
func (s *Session) onStatus(st boardlink.Status) {
	if s.configPending {
		s.configPending = false
		s.configuredOnce = true
		s.logger.Info("picolink: board configured", "boot_id", s.bootID, "boot_id_known", s.haveBoot,
			"state", st.State.String())
	}
	if st.Faults&boardlink.FaultRejectedCommand != 0 {
		// WARN, like the Zero's loop on each rejected command.
		s.logger.Warn("picolink: board rejected a non-finite command, treating it as missing")
	}
	if rising := st.Faults &^ s.lastFaults; rising&boardlink.FaultActuator != 0 {
		s.logger.Error("picolink: board reports an actuator write failure", "state", st.State.String())
	}
	s.lastFaults = st.Faults

	if err := s.status.Publish(MotorStatusFor(st)); err != nil {
		s.logger.Error("picolink: publishing MotorStatus", "error", err)
	}
}

// onOdometry converts a raw count into JointStates with
// pkg/portable/quadrature, the conversion the Zero's encoder runs. The
// estimator's interval is the board's own clock difference, which is the
// time the counts were actually taken over rather than when they arrived.
func (s *Session) onOdometry(o boardlink.Odometry) {
	if s.encoder == nil {
		return
	}
	counts := o.Counts
	if s.encoder.Invert {
		counts = -counts
	}

	if s.haveOdo && o.BoardTimeUS < s.lastOdoUS {
		// The board clock went backwards: a reset whose Hello has not been
		// seen yet. Start the estimator over rather than differencing
		// across boots.
		s.resetOdometry()
	}
	dtS := 0.0
	if s.haveOdo {
		dtS = min(max(float64(o.BoardTimeUS-s.lastOdoUS)/microsPerSecond, minOdometryDTS), maxOdometryDTS)
	}
	s.lastOdoUS, s.haveOdo = o.BoardTimeUS, true

	// The first Update after a reset only seeds the estimator, whatever dt.
	rpm := s.estimator.Update(counts, max(dtS, minOdometryDTS))
	revolutions, err := quadrature.CountsToRevolutions(counts, s.encoder.CountsPerRev)
	if err != nil {
		s.logger.Error("picolink: converting encoder counts", "error", err)
		return
	}
	if pubErr := s.joints.Publish(jointStatesFor(revolutions, rpm)); pubErr != nil {
		s.logger.Error("picolink: publishing JointStates", "error", pubErr)
	}
}

// onPong updates the clock estimate. A Pong echoing a host time later than
// now did not come from this session's Pings (a stale or corrupted echo)
// and is ignored.
func (s *Session) onPong(p boardlink.Pong, now time.Time) {
	recv := s.hostMicros(now)
	if p.HostTimeUS > recv {
		s.logger.Debug("picolink: ignoring a pong that echoes a future host time",
			"host_time_us", p.HostTimeUS, "now_us", recv)
		return
	}
	rtt, offset := clockFrom(p.HostTimeUS, recv, p.BoardTimeUS)
	est := ClockSync{Epoch: s.epoch, RTT: rtt, OffsetUS: offset, At: now}

	s.clockMu.Lock()
	s.clock, s.haveClock = est, true
	s.clockMu.Unlock()

	s.logger.Debug("picolink: clock", "rtt", rtt, "offset_us", offset)
}

// checkLink reports the link lost once per silence longer than
// linkTimeout: a WARN, and a FAULT MotorStatus so a consumer that only
// watches the subject sees it too.
func (s *Session) checkLink(now time.Time) {
	silence := now.Sub(s.lastFrameAt)
	if s.linkLost || silence < s.linkTimeout {
		return
	}
	s.linkLost = true
	s.logger.Warn("picolink: no frame from the board, still listening (its own command timeout stops the car)",
		"silence", silence, "dropped_frames", s.dropped.Load())
	if err := s.status.Publish(linkLostStatus(silence)); err != nil {
		s.logger.Error("picolink: publishing MotorStatus", "error", err)
	}
}

// pollButton samples the button evaluator on the host clock, as the Zero's
// button driver polls its GPIO. The board sent only the raw state; the
// debounce and hold thresholds are applied here, and a stalled link cannot
// advance a hold because the sample time is the host's, not the board's.
func (s *Session) pollButton(now time.Time) {
	if s.buttonEval == nil {
		return
	}
	if event := s.buttonEval.Sample(s.buttonPressed, now); event != nil {
		if err := s.buttonPub.Publish(nodebutton.EventMessageFor(*event)); err != nil {
			s.logger.Error("picolink: publishing ButtonEvent", "error", err)
		}
	}
	s.publishButtonHold(now)
}

// publishButtonHold tells the display how long the button has been held and
// what comes next, the Go analog of button_node.py's
// _publish_hold_progress: published every tick while the button is down,
// plus one final empty frame on release so the display clears instead of
// freezing on the last number.
func (s *Session) publishButtonHold(now time.Time) {
	held, pressed := s.buttonEval.Held(now)
	if !pressed {
		if s.buttonWasPressed {
			s.buttonWasPressed = false
			if err := s.buttonHoldPub.Publish(nodebutton.HoldMessageFor(0, nil)); err != nil {
				s.logger.Error("picolink: publishing ButtonHold", "error", err)
			}
		}
		return
	}
	s.buttonWasPressed = true
	thresholds := []nodebutton.HoldThreshold{
		{At: s.buttonThresholds.LongPressThreshold, Kind: "long"},
		{At: s.buttonThresholds.ShutdownPressThreshold, Kind: "shutdown"},
	}
	if err := s.buttonHoldPub.Publish(nodebutton.HoldMessageFor(held, thresholds)); err != nil {
		s.logger.Error("picolink: publishing ButtonHold", "error", err)
	}
}

// resetOdometry forgets the odometry history, for a board whose count
// restarted.
func (s *Session) resetOdometry() {
	s.haveOdo = false
	if s.estimator != nil {
		s.estimator.Reset()
	}
}

// hostMicros is t in microseconds since the session's epoch, on the
// monotonic clock.
func (s *Session) hostMicros(t time.Time) uint64 {
	return uint64(max(t.Sub(s.epoch).Microseconds(), 0))
}

// send encodes p with the next sequence number and writes it. The first
// frame of a session is prefixed with a delimiter, which flushes a board
// decoder that was left mid-frame (boardlink ignores empty frames).
func (lw *linkWriter) send(p *boardlink.Packet) error {
	lw.buf = lw.buf[:0]
	if !lw.started {
		lw.buf = append(lw.buf, 0)
		lw.started = true
	}
	p.Seq = lw.seq
	lw.seq++

	var err error
	if lw.buf, err = boardlink.Append(lw.buf, p); err != nil {
		return fmt.Errorf("picolink: encoding %s: %w", p.Type, err)
	}
	if _, err = lw.w.Write(lw.buf); err != nil {
		return fmt.Errorf("picolink: writing %s: %w", p.Type, err)
	}
	return nil
}

// HostTime converts a board timestamp into host time with this estimate.
func (c ClockSync) HostTime(boardUS uint64) time.Time {
	return c.Epoch.Add(time.Duration(int64(boardUS)-c.OffsetUS) * time.Microsecond)
}

// clockFrom is the SNTP-style estimate from one round trip: the board
// stamped its clock at some point between hostSend and hostRecv, taken as
// the midpoint.
func clockFrom(hostSendUS, hostRecvUS, boardUS uint64) (rtt time.Duration, offsetUS int64) {
	rttUS := hostRecvUS - hostSendUS
	mid := hostSendUS + rttUS/2
	return time.Duration(rttUS) * time.Microsecond, int64(boardUS) - int64(mid)
}

// linkLostStatus is the MotorStatus published when the link goes silent.
func linkLostStatus(silence time.Duration) *actuationv1.MotorStatus {
	st := MotorStatusFor(boardlink.Status{State: boardlink.StateFault})
	st.Detail = fmt.Sprintf("link lost: no frame from the board for %s", silence.Round(time.Millisecond))
	return st
}
