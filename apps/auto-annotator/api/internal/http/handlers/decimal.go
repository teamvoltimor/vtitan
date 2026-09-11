package handlers

import (
	"strconv"
)

// coordFromStr parses a decimal-string coordinate from the API wire format into
// a float64 for internal use (YOLO labels, gRPC).
func coordFromStr(s string) (float64, error) {
	return strconv.ParseFloat(s, 64)
}
