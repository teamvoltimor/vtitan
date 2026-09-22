//go:build cgo && linux && arm64

package recording

// Under the `cgo` tag, the video backend is the gocv mp4v sink (matches
// Python's video_recorder.py). The MJPEG fallback in sink_mjpeg.go is excluded
// by this same tag.
func newMJPEGAVISinkOrCgo(path string, fps float64) VideoSink {
	return newGocvVideoSink(path, fps)
}
