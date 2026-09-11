package recording

import (
	"fmt"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

// metadataFileName is the sidecar rosbag2 looks for when it is handed a
// DIRECTORY. Without it the run directory is not a bag as far as rosbag2 is
// concerned ("No storage could be initialized for the input URI"), and every
// diag_bag_*.py script -- all of which take a bag_dir -- has to be pointed at
// the .mcap file instead.
const metadataFileName = "metadata.yaml"

// metadataVersion is the rosbag2_bagfile_information schema version the
// pulled hardware bags carry, so a sim bag and a track bag present the same
// shape to the same reader.
const metadataVersion = 9

// rosDistro is recorded for provenance only; rosbag2 does not gate on it.
const rosDistro = "kilted"

// topicInfo accumulates what metadata.yaml has to state about one topic.
// Only CDR topics are described: a protobuf channel has no ROS type name to
// declare, and rosbag2 refuses a bag with mixed serialization formats
// anyway.
type topicInfo struct {
	typeName string
	count    int
}

// writeMetadata emits the rosbag2 sidecar. Called from Close, when the
// message counts and the time span are known -- none of it can be written up
// front, which is why the file appears only on a cleanly closed run.
func (r *RunRecorder) writeMetadata() error {
	if len(r.topics) == 0 {
		return nil
	}

	bagFile := r.stem + "_0.mcap"
	duration := r.maxLogTime - r.minLogTime
	total := 0
	names := make([]string, 0, len(r.topics))
	for name, info := range r.topics {
		total += info.count
		names = append(names, name)
	}
	// Sorted so two runs of the same scenario produce byte-identical
	// metadata, which makes the files diffable.
	sort.Strings(names)

	var b strings.Builder
	b.WriteString("rosbag2_bagfile_information:\n")
	fmt.Fprintf(&b, "  version: %d\n", metadataVersion)
	b.WriteString("  storage_identifier: mcap\n")
	b.WriteString("  duration:\n")
	fmt.Fprintf(&b, "    nanoseconds: %d\n", duration)
	b.WriteString("  starting_time:\n")
	fmt.Fprintf(&b, "    nanoseconds_since_epoch: %d\n", r.minLogTime)
	fmt.Fprintf(&b, "  message_count: %d\n", total)
	b.WriteString("  topics_with_message_count:\n")
	for _, name := range names {
		info := r.topics[name]
		b.WriteString("    - topic_metadata:\n")
		fmt.Fprintf(&b, "        name: %s\n", name)
		fmt.Fprintf(&b, "        type: %s\n", info.typeName)
		fmt.Fprintf(&b, "        serialization_format: %s\n", ROS2MessageEncoding)
		// Empty rather than fabricated. The QoS a topic was OFFERED with is
		// a property of the live publisher; a simulated run had none, and
		// inventing "reliable, keep_last 10" would state a fact about a
		// node that never existed.
		b.WriteString("        offered_qos_profiles: []\n")
		// REQUIRED by the v9 schema -- rosbag2 rejects the whole file with
		// "invalid node; first invalid key: type_description_hash" when it
		// is absent. Empty rather than computed: the value hashes the
		// message's type description and this recorder has no IDL to derive
		// one from. Readers here use the .msg text embedded in the MCAP
		// schema instead, so an empty hash costs nothing, while a fabricated
		// one would assert a type identity nothing checks.
		b.WriteString("        type_description_hash: \"\"\n")
		fmt.Fprintf(&b, "      message_count: %d\n", info.count)
	}
	b.WriteString("  compression_format: \"\"\n")
	b.WriteString("  compression_mode: \"\"\n")
	b.WriteString("  relative_file_paths:\n")
	fmt.Fprintf(&b, "    - %s\n", bagFile)
	b.WriteString("  files:\n")
	fmt.Fprintf(&b, "    - path: %s\n", bagFile)
	b.WriteString("      starting_time:\n")
	fmt.Fprintf(&b, "        nanoseconds_since_epoch: %d\n", r.minLogTime)
	b.WriteString("      duration:\n")
	fmt.Fprintf(&b, "        nanoseconds: %d\n", duration)
	fmt.Fprintf(&b, "      message_count: %d\n", total)
	b.WriteString("  custom_data: ~\n")
	fmt.Fprintf(&b, "  ros_distro: %s\n", rosDistro)

	path := filepath.Join(r.dir, metadataFileName)
	if err := os.WriteFile(path, []byte(b.String()), 0o600); err != nil {
		return fmt.Errorf("recording: writing %s: %w", path, err)
	}
	return nil
}
