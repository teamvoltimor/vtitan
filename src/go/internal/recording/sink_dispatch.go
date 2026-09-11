package recording

import (
	"fmt"

	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/src/go/internal/schema/protoschema"
)

// newVideoSink returns the video encoder for the build: the gocv mp4v backend
// under the `cgo` tag, otherwise the pure-Go MJPEG AVI sink. Same signature so
// RunRecorder is agnostic to which one is compiled in. The cgo variant is
// provided by sink_cgo.go (build-tagged); without the tag this resolves to the
// MJPEG sink.
func newVideoSink(path string, fps float64) VideoSink {
	return newMJPEGAVISinkOrCgo(path, fps)
}

// fileDescriptorSet returns the FileDescriptorSet for msg's schema so MCAP
// protobuf readers -- including EXTERNAL ones like Foxglove Studio, which
// has no access to this module's generated Go types -- can decode it. Our
// own bagreplay reader carries the generated types and unmarshals by type
// regardless, so this is specifically what makes a bag portable outside
// this repo.
func fileDescriptorSet(msg proto.Message) ([]byte, error) {
	set, err := protoschema.FileDescriptorSet(msg)
	if err != nil {
		return nil, fmt.Errorf("recording: building file descriptor set: %w", err)
	}
	return set, nil
}
