package recording

import (
	"maps"
	"os"
	"path/filepath"
	"testing"

	"github.com/foxglove/mcap/go/mcap"
)

// A metadata record written during the run is in the closed bag, readable
// by name, which is how a bag says what a test or a simulation injected.
func TestRunRecorder_WriteMetadataRecord(t *testing.T) {
	t.Parallel()

	rec, err := NewRun(t.TempDir(), RunOptions{Name: "faulty"})
	if err != nil {
		t.Fatalf("NewRun: %v", err)
	}
	want := map[string]string{"seed": "9", "vtitan.sensor.v1.scan": "delay=40ms"}
	if err = rec.WriteMetadata("nats_faults", want); err != nil {
		t.Fatalf("WriteMetadata: %v", err)
	}
	if err = rec.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}

	f, err := os.Open(filepath.Join(rec.Dir(), "faulty_0.mcap"))
	if err != nil {
		t.Fatal(err)
	}
	defer f.Close()
	reader, err := mcap.NewReader(f)
	if err != nil {
		t.Fatalf("NewReader: %v", err)
	}
	info, err := reader.Info()
	if err != nil {
		t.Fatalf("Info: %v", err)
	}
	if len(info.MetadataIndexes) != 1 || info.MetadataIndexes[0].Name != "nats_faults" {
		t.Fatalf("metadata indexes = %+v, want one named nats_faults", info.MetadataIndexes)
	}
	got, err := reader.GetMetadata(info.MetadataIndexes[0].Offset)
	if err != nil {
		t.Fatalf("GetMetadata: %v", err)
	}
	if !maps.Equal(got.Metadata, want) {
		t.Errorf("metadata = %v, want %v", got.Metadata, want)
	}
}

func TestRunRecorder_WriteMetadataAfterCloseFails(t *testing.T) {
	t.Parallel()

	rec, err := NewRun(t.TempDir(), RunOptions{Name: "closed"})
	if err != nil {
		t.Fatalf("NewRun: %v", err)
	}
	if err = rec.Open(); err != nil {
		t.Fatalf("Open: %v", err)
	}
	if err = rec.Close(); err != nil {
		t.Fatalf("Close: %v", err)
	}
	if err = rec.WriteMetadata("late", nil); err == nil {
		t.Error("WriteMetadata after Close: error = nil, want an error")
	}
}
