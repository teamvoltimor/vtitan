package bagreplay_test

import (
	"os"
	"path/filepath"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/test/bagreplay"
)

// bagDir locates a recorded run to replay. Bags are large and untracked, so
// this skips rather than fails when none is present -- CI and a fresh clone
// must not go red over a missing local artifact.
//
// VTITAN_BAG_DIR overrides the default, which is the newest run under
// platform/robot/vtitan_runs_pulled.
func bagDir(t *testing.T) string {
	t.Helper()

	if override := os.Getenv("VTITAN_BAG_DIR"); override != "" {
		return override
	}

	root := filepath.Join("..", "..", "..", "robot", "vtitan_runs_pulled")
	entries, err := os.ReadDir(root)
	if err != nil {
		t.Skipf("no recorded runs at %s: %v", root, err)
	}

	newest := ""
	for _, entry := range entries {
		// Run directories are named run_<timestamp>, so lexical order is
		// chronological order; the directory also holds loose log/pcap files.
		if entry.IsDir() && entry.Name() > newest {
			newest = entry.Name()
		}
	}
	if newest == "" {
		t.Skipf("no run directories under %s", root)
	}
	return filepath.Join(root, newest)
}

// TestReadNavDebug_RealBag is the end-to-end check that this package can
// actually read what the robot records: a real MCAP file, the real
// std_msgs/String envelope, and pydantic's real JSON. The synthetic CDR
// tests cannot catch a wrong assumption about any of those three.
func TestReadNavDebug_RealBag(t *testing.T) {
	t.Parallel()

	rows, err := bagreplay.ReadNavDebug(bagDir(t))
	if err != nil {
		t.Fatalf("ReadNavDebug() error = %v, want nil", err)
	}
	if len(rows) == 0 {
		t.Fatal("read 0 /nav_debug rows, want the per-tick stream the navigator publishes")
	}

	// Timestamps must be non-decreasing: they are the basis for lining a
	// replay up against any other topic in the same bag.
	for i := 1; i < len(rows); i++ {
		if rows[i].ElapsedS < rows[i-1].ElapsedS {
			t.Fatalf("row %d elapsed %v < row %d elapsed %v", i, rows[i].ElapsedS, i-1, rows[i-1].ElapsedS)
		}
	}

	// Every snapshot carries the phase that produced it; an empty one means
	// the JSON parsed into the wrong shape rather than that the field was
	// absent, since Python defaults it to NOT_YET_STEPPED.
	for i, row := range rows {
		if row.Snapshot.Phase == "" {
			t.Fatalf("row %d has an empty phase, want the branch name step() recorded", i)
		}
	}

	t.Logf("decoded %d /nav_debug rows spanning %.1fs", len(rows), rows[len(rows)-1].ElapsedS)
}
