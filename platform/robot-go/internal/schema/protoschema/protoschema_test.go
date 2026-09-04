package protoschema_test

import (
	"testing"
	"time"

	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protodesc"
	"google.golang.org/protobuf/reflect/protoreflect"
	"google.golang.org/protobuf/types/descriptorpb"
	"google.golang.org/protobuf/types/dynamicpb"
	"google.golang.org/protobuf/types/known/timestamppb"

	sensorv1 "github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/pb/vtitan/sensor/v1"
	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/schema/protoschema"
)

// TestFileDescriptorSet_DecodesWithoutTheGeneratedType is the real
// contract: an EXTERNAL tool with no access to this module's generated Go
// types -- exactly Foxglove Studio's position, and any MCAP reader other
// than this repo's own bagreplay -- must be able to fully decode a message
// using ONLY the returned FileDescriptorSet bytes plus the raw wire bytes.
//
// Imu was chosen because it imports google/protobuf/timestamp.proto (a
// cross-file, WELL-KNOWN-TYPE dependency) in addition to its own file --
// a descriptor set missing that import cannot resolve the Stamp field.
func TestFileDescriptorSet_DecodesWithoutTheGeneratedType(t *testing.T) {
	t.Parallel()

	orig := &sensorv1.Imu{
		Stamp: timestamppb.New(time.Unix(1234567890, 0)),
		Orientation: &sensorv1.Quaternion{
			X: 0.1, Y: 0.2, Z: 0.3, W: 0.9,
		},
		AngularVelocity: &sensorv1.Vector3{X: 0.01, Y: 0.02, Z: 0.03},
	}
	wire, err := proto.Marshal(orig)
	if err != nil {
		t.Fatalf("proto.Marshal(orig): %v", err)
	}

	descriptorSetBytes, err := protoschema.FileDescriptorSet(orig)
	if err != nil {
		t.Fatalf("FileDescriptorSet: %v", err)
	}

	// Rebuild a Files registry from ONLY the serialized bytes -- no import
	// of the sensorv1 package's compiled-in descriptor.
	var fdSet descriptorpb.FileDescriptorSet
	if err = proto.Unmarshal(descriptorSetBytes, &fdSet); err != nil {
		t.Fatalf("proto.Unmarshal(FileDescriptorSet): %v", err)
	}
	files, err := protodesc.NewFiles(&fdSet)
	if err != nil {
		t.Fatalf("protodesc.NewFiles: %v", err)
	}

	fullName := orig.ProtoReflect().Descriptor().FullName()
	desc, err := files.FindDescriptorByName(fullName)
	if err != nil {
		t.Fatalf("FindDescriptorByName(%s): %v", fullName, err)
	}
	md, ok := desc.(protoreflect.MessageDescriptor)
	if !ok {
		t.Fatalf("descriptor for %s is a %T, want protoreflect.MessageDescriptor", fullName, desc)
	}

	dyn := dynamicpb.NewMessage(md)
	if err = proto.Unmarshal(wire, dyn); err != nil {
		t.Fatalf("proto.Unmarshal(wire) into dynamicpb message: %v", err)
	}

	orientation := dyn.Get(md.Fields().ByName("orientation")).Message()
	if !orientation.IsValid() {
		t.Fatal("decoded dynamic message has no orientation field, want the nested Quaternion")
	}
	xField := orientation.Descriptor().Fields().ByName("x")
	if got := orientation.Get(xField).Float(); got < 0.099 || got > 0.101 {
		t.Errorf("orientation.x = %v, want ~0.1", got)
	}

	stamp := dyn.Get(md.Fields().ByName("stamp")).Message()
	if !stamp.IsValid() {
		t.Fatal("decoded dynamic message has no stamp field, want the cross-file Timestamp")
	}
	secondsField := stamp.Descriptor().Fields().ByName("seconds")
	if got := stamp.Get(secondsField).Int(); got != 1234567890 {
		t.Errorf("stamp.seconds = %v, want 1234567890", got)
	}
}
