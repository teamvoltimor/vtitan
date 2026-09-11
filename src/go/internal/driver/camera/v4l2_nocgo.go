package camera

import "errors"

// newV4L2Driver reports that the binary was built without the `cgo` tag. This
// file builds when NOT cgo-tagged, so SourceV4L2 is unavailable; the real
// implementation lives in v4l2_cgo.go (//go:build cgo && linux && arm64).
func newV4L2Driver(cfg Config) (Driver, error) {
	return nil, errors.New(
		"camera: SourceV4L2 requires the `cgo` build tag (gocv/OpenCV); rebuild with -tags cgo on linux/arm64",
	)
}
