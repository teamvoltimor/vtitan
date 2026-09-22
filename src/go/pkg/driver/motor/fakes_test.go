package motor_test

import (
	"context"
	"fmt"
	"sync"
)

// callLog records calls made through the fakes below, in order, so a test
// can assert both what was written and the sequence it was written in --
// the sequence is the actual thing under test for the connect-ordering
// safety property (see controller_test.go).
type callLog struct {
	mu    sync.Mutex
	calls []string
}

// fakeDutyWriter is a fake dutyWriter (motor.Controller's rpwm/lpwm
// dependency) that records every write and the last duty it was set to, so
// tests can assert on both order and end state without any real
// /sys/class/pwm or GPIO access.
type fakeDutyWriter struct {
	name string
	log  *callLog

	mu   sync.Mutex
	duty float64
}

// fakeEnableWriter is a fake enableWriter (motor.Controller's rEn/lEn
// dependency).
type fakeEnableWriter struct {
	name string
	log  *callLog

	mu   sync.Mutex
	high bool
}

func newFakeDutyWriter(name string, log *callLog) *fakeDutyWriter {
	return &fakeDutyWriter{name: name, log: log}
}

func newFakeEnableWriter(name string, log *callLog) *fakeEnableWriter {
	return &fakeEnableWriter{name: name, log: log}
}

func (l *callLog) record(entry string) {
	l.mu.Lock()
	defer l.mu.Unlock()
	l.calls = append(l.calls, entry)
}

func (l *callLog) entries() []string {
	l.mu.Lock()
	defer l.mu.Unlock()
	return append([]string(nil), l.calls...)
}

func (f *fakeDutyWriter) SetDuty(_ context.Context, fraction float64) error {
	f.log.record(fmt.Sprintf("%s.SetDuty(%.3f)", f.name, fraction))
	f.mu.Lock()
	f.duty = fraction
	f.mu.Unlock()
	return nil
}

func (f *fakeDutyWriter) lastDuty() float64 {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.duty
}

func (f *fakeEnableWriter) SetHigh(_ context.Context, high bool) error {
	f.log.record(fmt.Sprintf("%s.SetHigh(%v)", f.name, high))
	f.mu.Lock()
	f.high = high
	f.mu.Unlock()
	return nil
}

func (f *fakeEnableWriter) isHigh() bool {
	f.mu.Lock()
	defer f.mu.Unlock()
	return f.high
}
