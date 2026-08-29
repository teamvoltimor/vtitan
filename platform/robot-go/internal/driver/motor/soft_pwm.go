//go:build linux

package motor

import (
	"context"
	"math"
	"sync/atomic"
	"time"

	"github.com/warthog618/go-gpiocdev"
)

// softPWM is the real dutyWriter (controller.go) for LPWM (reverse):
// software-bit-banged PWM over a go-gpiocdev output line, since the Pi's
// two hardware PWM engines are both already claimed (one by the steering
// servo, one by RPWM/forward — see docs/bts7960-ibt2-wiring.md). Reverse is
// only ever used for low-duty parking/recovery, which tolerates software
// PWM's jitter in a way the steering servo's absolute-position hold does
// not — same tradeoff the Python driver documents for its gpiozero
// PWMOutputDevice equivalent.
type softPWM struct {
	line   *gpiocdev.Line
	period time.Duration

	dutyBits atomic.Uint64 // math.Float64bits of the current duty fraction
	stopCh   chan struct{}
	doneCh   chan struct{}
}

var _ dutyWriter = (*softPWM)(nil)

// newSoftPWM builds a softPWM driving line at frequencyHz, initially at 0
// duty. Call start to begin toggling the line.
func newSoftPWM(line *gpiocdev.Line, frequencyHz int) *softPWM {
	return &softPWM{
		line:   line,
		period: time.Second / time.Duration(frequencyHz),
		stopCh: make(chan struct{}),
		doneCh: make(chan struct{}),
	}
}

// start begins the toggle goroutine. Must be called at most once.
func (s *softPWM) start() {
	go s.run()
}

// SetDuty stores the target duty fraction ([0, 1]); the toggle goroutine
// picks it up on its next cycle. Lock-free by design — the toggle loop
// reads this on every cycle (up to frequencyHz times per second) and a
// mutex would serialize it against SetDuty for no benefit, since the
// consumer only ever wants the latest value, never a queue of past ones.
func (s *softPWM) SetDuty(_ context.Context, fraction float64) error {
	s.dutyBits.Store(math.Float64bits(fraction))
	return nil
}

// Stop halts the toggle goroutine and drives the line LOW.
func (s *softPWM) Stop() error {
	close(s.stopCh)
	<-s.doneCh
	if err := s.line.SetValue(gpioLow); err != nil {
		return err //nolint:wrapcheck // sole caller (driver.go's Close) wraps with its own context
	}
	return nil
}

func (s *softPWM) run() {
	defer close(s.doneCh)
	for {
		duty := math.Float64frombits(s.dutyBits.Load())
		high := time.Duration(duty * float64(s.period))
		low := s.period - high

		if high > 0 {
			_ = s.line.SetValue(gpioHigh)
			if s.wait(high) {
				return
			}
		}
		if low > 0 {
			_ = s.line.SetValue(gpioLow)
			if s.wait(low) {
				return
			}
		}
	}
}

// wait blocks for d or until Stop is called, reporting whether Stop fired.
func (s *softPWM) wait(d time.Duration) bool {
	timer := time.NewTimer(d)
	defer timer.Stop()
	select {
	case <-s.stopCh:
		return true
	case <-timer.C:
		return false
	}
}
