package handlers

import (
	"strconv"
)

// coordFromStr parses a decimal-string coordinate from the API wire format into
// a float64 for internal use (YOLO labels, gRPC).
func coordFromStr(s string) (float64, error) {
	return strconv.ParseFloat(s, 64)
}

// coordToStr formats a float64 coordinate into a decimal string for the API
// wire format.
func coordToStr(f float64) string {
	return strconv.FormatFloat(f, 'f', -1, 64)
}
