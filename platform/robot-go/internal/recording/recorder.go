package recording

import (
	"fmt"
	"os"
	"path/filepath"
	"sync"
	"time"

	"github.com/foxglove/mcap/go/mcap"
	"google.golang.org/protobuf/proto"
)

// RunRecorder owns a single run directory and fans frames/topics out to the
// MCAP bag, the debug video, and the periodic photos. It mirrors the Python
// contract: one `run_<stamp>/` holds `run_<stamp>_0.mcap`, `video.mp4`, and
// `captures/`. The bag recorder "owns" the directory; the video and photo
// writers only poll for it (dataset_capture.py:64).
type RunRecorder struct {
	mu            sync.Mutex
	dir           string
	stamp         string
	mcapW         *mcap.Writer
	mcapF         *os.File
	video         *VideoWriter
	photos        *PhotoCapture
	started       bool
	channels      map[string]uint16
	nextChannelID uint16
	nextSchemaID  uint16
}

// RunOptions configures a new run.
type RunOptions struct {
	// Now is the run start time; defaults to time.Now if zero.
	Now time.Time
	// VideoFPS sizes the video encoder (only used by the gocv backend).
	VideoFPS float64
	// PhotoInterval / PhotoSubdir / PhotoRequireDetection configure the periodic
	// dataset capture (Go port of dataset_capture.py).
	PhotoInterval   time.Duration
	PhotoSubdir     string
	PhotoRequireDet bool
	// Video toggles the debug video; some runs may record bag-only.
	Video bool
}

// NewRun creates (but does not open) a recorder for a fresh run directory under
// runsRoot, named run_<stamp>. Call Open to start capturing.
func NewRun(runsRoot string, opts RunOptions) (*RunRecorder, error) {
	now := opts.Now
	if now.IsZero() {
		now = time.Now()
	}
	stamp := now.Format("20060102_150405")
	dir := filepath.Join(runsRoot, "run_"+stamp)
	if err := os.MkdirAll(dir, 0o755); err != nil {
		return nil, fmt.Errorf("recording: creating run dir %s: %w", dir, err)
	}
	r := &RunRecorder{
		dir:          dir,
		stamp:        stamp,
		channels:     make(map[string]uint16),
		nextSchemaID: 1,
	}
	if opts.Video {
		// The gocv sink is selected at build time under the `cgo` tag; otherwise
		// newVideoSink falls back to the pure-Go MJPEG AVI sink.
		r.video = NewVideoWriter(newVideoSink(filepath.Join(dir, "video.mp4"), opts.VideoFPS))
	}
	subdir := opts.PhotoSubdir
	if subdir == "" {
		subdir = "captures"
	}
	r.photos = NewPhotoCapture(opts.PhotoInterval, subdir, opts.PhotoRequireDet)
	return r, nil
}

// Dir returns the run directory. Valid after Open (or NewRun).
func (r *RunRecorder) Dir() string { return r.dir }

// Open creates the MCAP bag file inside the run directory. Idempotent.
func (r *RunRecorder) Open() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	if r.started {
		return nil
	}
	bagPath := filepath.Join(r.dir, "run_"+r.stamp+"_0.mcap")
	f, err := os.Create(bagPath)
	if err != nil {
		return fmt.Errorf("recording: creating bag %s: %w", bagPath, err)
	}
	w, err := mcap.NewWriter(f, &mcap.WriterOptions{
		IncludeCRC: true,
	})
	if err != nil {
		f.Close()
		return fmt.Errorf("recording: creating mcap writer: %w", err)
	}
	// A minimal well-formed MCAP needs a header + at least one channel before
	// messages; we register channels lazily in WriteMessage, but must emit the
	// header now so the file is valid even with zero messages.
	if err = w.WriteHeader(&mcap.Header{Profile: "vtitan", Library: "robot-go"}); err != nil {
		f.Close()
		return fmt.Errorf("recording: writing mcap header: %w", err)
	}
	r.mcapF = f
	r.mcapW = w
	r.started = true
	return nil
}

// WriteMessage appends a protobuf message to the MCAP bag under subject. The
// schema is derived from the message's protobuf descriptor, registered once per
// subject. This is how the Go recorder mirrors the Python rosbag2 topics.
func (r *RunRecorder) WriteMessage(subject string, msg proto.Message, logTime uint64) error {
	// Ensure the bag is open before taking the lock, so Open() (which also locks
	// r.mu) is never called while the caller already holds it.
	if err := r.Open(); err != nil {
		return err
	}
	r.mu.Lock()
	defer r.mu.Unlock()
	schemaID, err := r.ensureSchema(subject, msg)
	if err != nil {
		return err
	}
	data, err := proto.Marshal(msg)
	if err != nil {
		return fmt.Errorf("recording: marshaling %s: %w", subject, err)
	}
	return r.mcapW.WriteMessage(&mcap.Message{
		ChannelID:   schemaID,
		Sequence:    0,
		LogTime:     logTime,
		PublishTime: logTime,
		Data:        data,
	})
}

// ensureSchema registers the subject's channel+schema in the MCAP if absent,
// returning the channel ID.
func (r *RunRecorder) ensureSchema(subject string, msg proto.Message) (uint16, error) {
	// MCAP channels are keyed by ID; we map subject->id in memory. One channel
	// per subject, registered lazily on first use.
	if id, ok := r.channels[subject]; ok {
		return id, nil
	}
	schema := &mcap.Schema{
		ID:       r.nextSchemaID,
		Name:     string(proto.MessageName(msg)),
		Encoding: "protobuf",
		Data:     mustProtoDescriptor(msg),
	}
	if err := r.mcapW.WriteSchema(schema); err != nil {
		return 0, fmt.Errorf("recording: writing schema %s: %w", subject, err)
	}
	r.nextSchemaID++
	id := r.nextChannelID
	r.nextChannelID++
	ch := &mcap.Channel{
		ID:              id,
		SchemaID:        schema.ID,
		Topic:           subject,
		MessageEncoding: "protobuf",
	}
	if err := r.mcapW.WriteChannel(ch); err != nil {
		return 0, fmt.Errorf("recording: writing channel %s: %w", subject, err)
	}
	r.channels[subject] = id
	return id, nil
}

// Video returns the run's video writer (nil if video was disabled).
func (r *RunRecorder) Video() *VideoWriter { return r.video }

// Photos returns the run's photo capture; call MaybeCapture on each frame tick.
func (r *RunRecorder) Photos() *PhotoCapture { return r.photos }

// Close finalizes the bag, video, and any open handles. Safe to call once.
func (r *RunRecorder) Close() error {
	r.mu.Lock()
	defer r.mu.Unlock()
	var firstErr error
	if r.video != nil {
		if err := r.video.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
	}
	if r.mcapW != nil {
		if err := r.mcapW.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
		r.mcapW = nil
	}
	if r.mcapF != nil {
		if err := r.mcapF.Close(); err != nil && firstErr == nil {
			firstErr = err
		}
		r.mcapF = nil
	}
	return firstErr
}
