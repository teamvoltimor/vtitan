package recording

// errSink is a VideoSink that always returns its construction error.
type errSink struct{ err error }

// newMJPEGAVISinkOrCgo selects the pure-Go MJPEG-in-AVI backend. Under the
// `cgo` tag, sink_cgo.go overrides this with the gocv mp4v sink.
func newMJPEGAVISinkOrCgo(path string, _ float64) VideoSink {
	s, err := newMJPEGAVISink(path)
	if err != nil {
		// VideoSink is an interface; surfacing the error by panicking is wrong
		// in a library. Instead return a sink that records the open error and
		// returns it on the first Write. Simpler: caller (RunRecorder) already
		// created the dir, so os.Create failing is exceptional; wrap in a
		// failingSink.
		return &errSink{err: err}
	}
	return s
}

func (e *errSink) Write(_ *Frame, _ HudOverlay) error {
	return e.err
}

func (e *errSink) Close() error {
	return e.err
}
