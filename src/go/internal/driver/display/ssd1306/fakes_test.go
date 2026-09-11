package ssd1306_test

import "fmt"

// fakeI2C is a fake i2cWriter (ssd1306.Controller's only dependency) that
// records every I2C write as a hex string, in order, so a test can assert
// both what was written and the sequence it was written in — no real I2C
// bus involved. Same "fake records calls, test asserts on the sequence"
// pattern as internal/driver/motor's fakes_test.go.
type fakeI2C struct {
	writes []string
}

func (f *fakeI2C) Write(b []byte) (int, error) {
	f.writes = append(f.writes, fmt.Sprintf("%X", b))
	return len(b), nil
}
