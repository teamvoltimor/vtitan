// Package picolink is the Pi 5 side of the Pico 2 actuation link
// (adr:0098-pico-actuation-board-and-portable-cores): a supervised loop that
// makes a Pico 2 on a serial port look, on NATS, exactly like the Zero's
// motor node (internal/node/motor), so nothing above the actuation board
// learns which board is fitted.
//
// The wire protocol is pkg/portable/boardlink. The loop:
//
//   - sends a Config built from the SAME sources the Zero reads (see
//     BoardConfig and LoadProfile) as soon as it starts, since an already
//     configured board sends no Hello, and again on every Hello, so a board
//     that resets mid-round is reconfigured by the same path as one that
//     just booted. A Config is checked with boardloop.ValidateConfig, the
//     board's own rule, before it is sent, and a Hello that answers a Config
//     from the same boot is logged as a refusal;
//   - forwards every AckermannCmd from vtitan.actuation.v1.ackermann_cmd as a
//     Command, unfiltered (see Session.Run for why non-finite values are
//     forwarded too);
//   - republishes Status as MotorStatus (MotorStatusFor) and Odometry as
//     JointStates, the latter converted with pkg/portable/quadrature, the code
//     the Zero's encoder runs;
//   - evaluates the board's raw button edges with the Zero's own evaluator
//     (pkg/driver/button) and publishes each resulting ButtonEvent on
//     vtitan.ui.v1.button_event, so a start button on the Pico reaches the
//     state machine exactly as the Zero's button does;
//   - pings once a second and keeps the latest round trip and clock offset
//     (Session.Clock), which is the Pico half of go-future.md section 4.6:
//     the board has no RTC and counts from boot, so its offset from the host
//     is estimated from the link's own round trip, with RTT/2 as its bound.
//
// Safety does not live here. The board runs the command watchdog and a
// hardware watchdog itself, so a lost link is a silent command stream the
// board already stops on. The host therefore does nothing unsafe by carrying
// on through a silence: it logs it, reports it as a FAULT MotorStatus, and
// keeps listening.
//
// Running this loop and the Zero's motor loop at once would double-publish
// MotorStatus, JointStates and ButtonEvent. cmd/pi5 enables it only when a
// port is given, and the deployment picks one board.
package picolink
