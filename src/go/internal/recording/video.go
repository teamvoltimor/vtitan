package recording

import (
	"image"
	"image/color"
	"sync"
)

// Frame is one captured image handed to the recorder. It is the same shape as
// camera.Frame (rgb8, contiguous rows) so a frame from any capture backend
// (v4l2/topic/synthetic) flows through unchanged.
type Frame struct {
	Width    int
	Height   int
	Stride   int
	Encoding string
	Data     []byte
}

// HudOverlay is the telemetry composited onto the video, mirroring Python's
// video_recorder.FrameSnapshot (nav_debug + scan + active_challenge). The pure-
// Go MJPEG backend ignores it for now (no HUD draw); the gocv backend draws it.
type HudOverlay struct {
	NavDebug        map[string]any
	ScanRanges      []float32
	ScanAngles      []float32
	ActiveChallenge string
}

// VideoSink encodes frames to a container file. Implementations must be
// drop-on-backpressure: if they cannot keep up, they drop rather than block the
// capture loop (video_recorder.py:30's _QUEUE_MAXSIZE contract).
type VideoSink interface {
	// Write encodes one frame. It must never block the caller for long; if the
	// encoder is behind, it may drop the frame and return nil.
	Write(f *Frame, hud HudOverlay) error
	// Close finalizes the container. Blocks briefly to flush; the file may be
	// unplayable if Close is skipped.
	Close() error
}

// VideoWriter owns a bounded frame queue and an encoder goroutine, mirroring
// video_recorder.py's threaded design. submit() never blocks the caller: when
// the queue is full the frame is dropped (counted in Dropped()).
type VideoWriter struct {
	queue   chan frameJob
	closed  chan struct{}
	once    sync.Once
	dropped int
	mu      sync.Mutex
	sink    VideoSink
}

type frameJob struct {
	f   *Frame
	hud HudOverlay
}

// QueueMax is the bounded queue depth, matching video_recorder.py's
// _QUEUE_MAXSIZE (3): a full queue means the encoder is genuinely behind, and
// the point is to notice and drop, not buffer minutes of frames.
const QueueMax = 3

// EncodingRGB8 is the raw 8-bit RGB pixel encoding Frame carries; every
// capture backend produces it and every sink expects it.
const EncodingRGB8 = "rgb8"

// NewVideoWriter starts the encoder goroutine around sink.
func NewVideoWriter(sink VideoSink) *VideoWriter {
	w := &VideoWriter{
		queue:  make(chan frameJob, QueueMax),
		closed: make(chan struct{}),
		sink:   sink,
	}
	go w.run()
	return w
}

// Submit hands a frame to the encoder. Drops it (and counts it) if the queue is
// full, never blocking the caller.
func (w *VideoWriter) Submit(f *Frame, hud HudOverlay) {
	select {
	case w.queue <- frameJob{f: f, hud: hud}:
	default:
		w.mu.Lock()
		w.dropped++
		w.mu.Unlock()
	}
}

// Dropped returns how many frames were dropped for backpressure so far.
func (w *VideoWriter) Dropped() int {
	w.mu.Lock()
	defer w.mu.Unlock()
	return w.dropped
}

// Close signals the encoder to finalize and waits for it to drain.
func (w *VideoWriter) Close() error {
	w.once.Do(func() {
		close(w.closed)
	})
	return nil
}

func (w *VideoWriter) run() {
	for {
		select {
		case <-w.closed:
			// Drain whatever is queued, then finalize.
			for {
				select {
				case job := <-w.queue:
					_ = w.sink.Write(job.f, job.hud)
				default:
					_ = w.sink.Close()
					return
				}
			}
		case job := <-w.queue:
			_ = w.sink.Write(job.f, job.hud)
		}
	}
}

// rgb8Image adapts an rgb8 Frame to image.Image for encoders that need one.
func (f *Frame) image() (image.Image, error) {
	return rgb8ToImage(f)
}

var _ = color.RGBAModel // keep image/color imported for rgb8Image users
