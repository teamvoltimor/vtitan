package bagreplay_test

import (
	"os"
	"path/filepath"
	"sort"
	"testing"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/recording"
	"github.com/teamvoltimor/vtitan/platform/robot-go/test/bagreplay"
)

// bagDir locates a recorded run to replay. Bags are large and untracked, so
// this skips rather than fails when none is present -- CI and a fresh clone
// must not go red over a missing local artifact.
//
// VTITAN_BAG_DIR overrides the default, which is the newest run under the
// repo-root data/live/runs tree (shared by the Python and Go stacks).
func bagDir(t *testing.T) string {
	t.Helper()

	if override := os.Getenv("VTITAN_BAG_DIR"); override != "" {
		return override
	}

	root, err := recording.RunsRoot()
	if err != nil {
		t.Skipf("no shared runs root: %v", err)
	}
	entries, err := os.ReadDir(root)
	if err != nil {
		t.Skipf("no recorded runs at %s: %v", root, err)
	}

	// Run directories are named run_<timestamp>, so lexical order is
	// chronological order; the directory also holds loose log/pcap files.
	var runs []string
	for _, entry := range entries {
		if entry.IsDir() {
			runs = append(runs, entry.Name())
		}
	}
	sort.Sort(sort.Reverse(sort.StringSlice(runs)))

	// Newest-first, but NOT newest-only: a run still being recorded (or one
	// whose process died) has no MCAP footer and fails to open with "invalid
	// magic at end of file". Observed for real -- a recording started while
	// this suite was being written became the newest directory and broke it.
	// Falling back keeps the test meaningful instead of going red on an
	// unrelated live recording.
	for _, run := range runs {
		dir := filepath.Join(root, run)
		if _, readErr := bagreplay.ReadNavDebug(dir); readErr != nil {
			// Logged rather than swallowed: if a decode regression makes
			// every bag unreadable, the skip below reports it instead of
			// the suite quietly passing.
			t.Logf("skipping unreadable run %s: %v", run, readErr)
			continue
		}
		return dir
	}

	t.Skipf("no readable run directories under %s (%d tried)", root, len(runs))
	return ""
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

// TestReadScan_RealBag is the end-to-end check that this package can decode
// the physical lidar's sensor_msgs/LaserScan off a real MCAP bag -- the other
// half of the parity gate, alongside TestReadNavDebug_RealBag.
func TestReadScan_RealBag(t *testing.T) {
	t.Parallel()

	rows, err := bagreplay.ReadScan(bagDir(t))
	if err != nil {
		t.Fatalf("ReadScan() error = %v, want nil", err)
	}
	if len(rows) == 0 {
		t.Fatal("read 0 /scan rows, want the lidar's per-sweep stream")
	}

	// Every decoded sweep must carry the angles the navigator consumes.
	for i, row := range rows {
		if len(row.Scan.RangesM) == 0 {
			t.Fatalf("row %d has 0 ranges, want a populated sweep", i)
		}
		if row.Scan.AngleIncrement == 0 {
			t.Fatalf("row %d has angle_increment 0, want the per-ray step", i)
		}
	}

	t.Logf("decoded %d /scan rows, first sweep %d rays over [%.4f, %.4f]",
		len(rows), len(rows[0].Scan.RangesM), rows[0].Scan.AngleMin, rows[0].Scan.AngleMax)
}
