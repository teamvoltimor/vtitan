package bagreplay

import (
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"

	"github.com/foxglove/mcap/go/mcap"
)

// NavDebugRow pairs a decoded snapshot with when it was recorded.
//
// ElapsedS is seconds since the bag's FIRST message rather than since this
// topic's first message, matching bag_io.read_nav_debug_rows -- so a
// timestamp here lines up with one taken from any other topic in the same
// bag.
type NavDebugRow struct {
	ElapsedS float64
	Snapshot NavDebugSnapshot
}

// ScanRow pairs a decoded LIDAR sweep with when it was recorded, on the same
// elapsed-time basis as NavDebugRow (the bag's first message on ANY topic).
type ScanRow struct {
	ElapsedS float64
	Scan     LaserScan
}

// NavDebugTopic is the topic CoreNavigator publishes its per-tick snapshot
// on, matching bag_io.Topics.NAV_DEBUG.
const NavDebugTopic = "/nav_debug"

// nanosPerSecond converts MCAP's nanosecond log timestamps to the seconds
// the diag_bag_*.py scripts report elapsed time in.
const nanosPerSecond = 1e9

// FindBagFile resolves a run directory to the .mcap file inside it, and
// passes a path that is already a file straight through.
//
// Runs are recorded as a directory containing a single split file
// (run_<stamp>/run_<stamp>_0.mcap), so callers naturally hold the directory
// name; requiring them to know the split-file convention would push that
// detail into every test.
func FindBagFile(path string) (string, error) {
	info, err := os.Stat(path)
	if err != nil {
		return "", fmt.Errorf("bagreplay: stat %s: %w", path, err)
	}
	if !info.IsDir() {
		return path, nil
	}

	matches, err := filepath.Glob(filepath.Join(path, "*.mcap"))
	if err != nil {
		return "", fmt.Errorf("bagreplay: globbing %s: %w", path, err)
	}
	switch len(matches) {
	case 0:
		return "", fmt.Errorf("bagreplay: no .mcap file in %s", path)
	case 1:
		return matches[0], nil
	default:
		// Multi-split runs would need concatenating in timestamp order;
		// nothing in this project records them yet, so refuse rather than
		// silently replay only the first split.
		return "", fmt.Errorf("bagreplay: %d .mcap files in %s, expected 1", len(matches), path)
	}
}

// FindVideo returns the path to a run's debug video, if the recorder wrote one
// (run_<stamp>/video.mp4). A run may have no video (bag-only recording), in
// which case the second return is false and the path is empty.
func FindVideo(runDir string) (string, bool) {
	p := filepath.Join(runDir, "video.mp4")
	if _, err := os.Stat(p); err != nil {
		return "", false
	}
	return p, true
}

// ListPhotos returns the sorted paths of a run's periodic dataset photos
// (run_<stamp>/captures/capture_NNNN.jpg), used for later annotation/correlation
// with the bag. An empty or missing captures dir yields an empty slice, not an
// error -- a run with no photos is valid.
func ListPhotos(runDir string) ([]string, error) {
	dir := filepath.Join(runDir, "captures")
	matches, err := filepath.Glob(filepath.Join(dir, "capture_*.jpg"))
	if err != nil {
		return nil, fmt.Errorf("bagreplay: globbing photos in %s: %w", dir, err)
	}
	return matches, nil
}

// ReadScan replays a bag and returns every /scan sweep in recorded order.
//
// path may be either the run directory or the .mcap file itself.
func ReadScan(path string) ([]ScanRow, error) {
	file, err := FindBagFile(path)
	if err != nil {
		return nil, err
	}

	handle, err := os.Open(file)
	if err != nil {
		return nil, fmt.Errorf("bagreplay: opening %s: %w", file, err)
	}
	defer handle.Close()

	reader, err := mcap.NewReader(handle)
	if err != nil {
		return nil, fmt.Errorf("bagreplay: reading %s as MCAP: %w", file, err)
	}
	defer reader.Close()

	iterator, err := reader.Messages()
	if err != nil {
		return nil, fmt.Errorf("bagreplay: iterating %s: %w", file, err)
	}

	var rows []ScanRow
	var firstStamp uint64
	haveFirst := false

	var scratch mcap.Message
	for {
		_, channel, message, nextErr := iterator.NextInto(&scratch)
		if errors.Is(nextErr, io.EOF) {
			break
		}
		if nextErr != nil {
			return nil, fmt.Errorf("bagreplay: reading next message from %s: %w", file, nextErr)
		}

		if !haveFirst {
			firstStamp, haveFirst = message.LogTime, true
		}
		if channel == nil || channel.Topic != ScanTopic {
			continue
		}

		scan, decodeErr := DecodeLaserScan(message.Data)
		if decodeErr != nil {
			return nil, fmt.Errorf("bagreplay: decoding %s at %d: %w", ScanTopic, message.LogTime, decodeErr)
		}
		rows = append(rows, ScanRow{
			ElapsedS: float64(message.LogTime-firstStamp) / nanosPerSecond,
			Scan:     scan,
		})
	}

	return rows, nil
}

// ReadNavDebug replays a bag and returns every /nav_debug snapshot in
// recorded order, matching bag_io.read_nav_debug_rows.
//
// path may be either the run directory or the .mcap file itself.
func ReadNavDebug(path string) ([]NavDebugRow, error) {
	file, err := FindBagFile(path)
	if err != nil {
		return nil, err
	}

	handle, err := os.Open(file)
	if err != nil {
		return nil, fmt.Errorf("bagreplay: opening %s: %w", file, err)
	}
	defer handle.Close()

	reader, err := mcap.NewReader(handle)
	if err != nil {
		return nil, fmt.Errorf("bagreplay: reading %s as MCAP: %w", file, err)
	}
	defer reader.Close()

	iterator, err := reader.Messages()
	if err != nil {
		return nil, fmt.Errorf("bagreplay: iterating %s: %w", file, err)
	}

	var rows []NavDebugRow
	// firstStamp is the bag's first message on ANY topic, so elapsed times
	// are comparable across topics -- see NavDebugRow.
	var firstStamp uint64
	haveFirst := false

	// NextInto reuses one Message rather than heap-allocating per record;
	// a full run is tens of thousands of messages.
	var scratch mcap.Message
	for {
		_, channel, message, nextErr := iterator.NextInto(&scratch)
		if errors.Is(nextErr, io.EOF) {
			break
		}
		if nextErr != nil {
			return nil, fmt.Errorf("bagreplay: reading next message from %s: %w", file, nextErr)
		}

		if !haveFirst {
			firstStamp, haveFirst = message.LogTime, true
		}
		if channel == nil || channel.Topic != NavDebugTopic {
			continue
		}

		snapshot, decodeErr := DecodeNavDebug(message.Data)
		if decodeErr != nil {
			return nil, fmt.Errorf("bagreplay: decoding %s at %d: %w", NavDebugTopic, message.LogTime, decodeErr)
		}
		rows = append(rows, NavDebugRow{
			ElapsedS: float64(message.LogTime-firstStamp) / nanosPerSecond,
			Snapshot: snapshot,
		})
	}

	return rows, nil
}
