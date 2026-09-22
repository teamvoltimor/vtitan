//go:build cgo && linux && arm64

package camera

import (
	"github.com/teamvoltimor/vtitan/src/go/internal/driver/camera/gocv"
)

// newV4L2Driver delegates to the gocv-backed V4L2 implementation, compiled only
// under the `cgo` build tag (see internal/driver/camera/gocv). Without the tag,
// SourceV4L2 is rejected by camera.New (see camera.go's default branch).
func newV4L2Driver(cfg Config) (Driver, error) {
	return gocv.NewV4L2(cfg)
}
