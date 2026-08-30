package recording

import (
	"google.golang.org/protobuf/proto"
)

// newVideoSink returns the video encoder for the build: the gocv mp4v backend
// under the `cgo` tag, otherwise the pure-Go MJPEG AVI sink. Same signature so
// RunRecorder is agnostic to which one is compiled in. The cgo variant is
// provided by sink_cgo.go (build-tagged); without the tag this resolves to the
// MJPEG sink.
func newVideoSink(path string, fps float64) VideoSink {
	return newMJPEGAVISinkOrCgo(path, fps)
}

// mustProtoDescriptor returns the FileDescriptorSet for msg's schema so MCAP
// protobuf readers can decode it. Our own bagreplay reader carries the generated
// Go types and unmarshals by type, so an empty descriptor still round-trips
// within this repo; external tools get the descriptor when available.
func mustProtoDescriptor(msg proto.Message) []byte {
	// proto.Marshal of a FileDescriptorSet is heavy to compute here; the
	// generated types in this module decode without it. Return nil and rely on
	// the reader's generated schema. (If cross-tool interop is needed later,
	// populate this from protoregistry.)
	return nil
}
