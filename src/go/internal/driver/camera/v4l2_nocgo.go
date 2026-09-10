// Builds when NOT cgo-tagged: SourceV4L2 is unavailable, so camera.New's v4l2
// branch reports that the binary was built without the `cgo` tag. The real
// implementation lives in v4l2_cgo.go (//go:build cgo && linux && arm64).
package camera

import "fmt"

func newV4L2Driver(cfg Config) (Driver, error) {
	return nil, fmt.Errorf("camera: SourceV4L2 requires the `cgo` build tag (gocv/OpenCV); rebuild with -tags cgo on linux/arm64")
}
