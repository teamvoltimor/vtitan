// Package protoschema builds the self-describing protobuf schema bytes
// (a serialized descriptorpb.FileDescriptorSet) an EXTERNAL tool needs to
// decode a message it did not compile against -- MCAP's "protobuf" schema
// encoding and the Foxglove WebSocket protocol's "protobuf" schema encoding
// both use exactly this format, so this package is the single source both
// internal/recording and internal/foxglove build their schema bytes from.
package protoschema

import (
	"fmt"

	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protodesc"
	"google.golang.org/protobuf/reflect/protoreflect"
	"google.golang.org/protobuf/types/descriptorpb"
)

// FileDescriptorSet returns the serialized descriptorpb.FileDescriptorSet
// for msg's type, INCLUDING every file it transitively imports. A decoder
// with only msg's own .proto file (no transitive dependencies) cannot
// resolve cross-file type references -- e.g. NavigatorDebug embedding
// NavigatorPose from a different file -- so this walks the full import
// graph rather than emitting a single FileDescriptorProto.
func FileDescriptorSet(msg proto.Message) ([]byte, error) {
	fd := msg.ProtoReflect().Descriptor().ParentFile()
	seen := make(map[string]bool)
	var files []*descriptorpb.FileDescriptorProto
	collectFiles(fd, seen, &files)

	set := &descriptorpb.FileDescriptorSet{File: files}
	data, err := proto.Marshal(set)
	if err != nil {
		return nil, fmt.Errorf("protoschema: marshaling FileDescriptorSet for %s: %w", msg.ProtoReflect().Descriptor().FullName(), err)
	}
	return data, nil
}

// collectFiles appends fd's descriptor and every file it imports (directly
// or transitively) to files, skipping ones already seen by path -- a
// message graph commonly re-imports the same shared file (e.g. a
// timestamp or common-types proto) from multiple message types.
func collectFiles(fd protoreflect.FileDescriptor, seen map[string]bool, files *[]*descriptorpb.FileDescriptorProto) {
	if seen[fd.Path()] {
		return
	}
	seen[fd.Path()] = true
	for i := range fd.Imports().Len() {
		collectFiles(fd.Imports().Get(i).FileDescriptor, seen, files)
	}
	*files = append(*files, protodesc.ToFileDescriptorProto(fd))
}
