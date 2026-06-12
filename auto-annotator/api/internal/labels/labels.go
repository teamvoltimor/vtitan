// Package labels reads YOLO label (.txt) files and converts their normalized
// coordinates into polygon point lists. It is the Go port of the read path in
// the Python label_store + coordinates modules.
package labels

import (
	"bufio"
	"os"
	"path/filepath"
	"strconv"
	"strings"
)

// Record is one annotation line from a YOLO label file: a class index followed
// by a flat list of normalized coordinates ([xc, yc, w, h] for detection or
// [x1, y1, x2, y2, ...] for segmentation).
type Record struct {
	ClassID int
	Coords  []float64
}

// Point is a normalized 2-D coordinate in the range [0, 1].
type Point struct {
	X float64
	Y float64
}

// Load reads and parses the label file at <labelsDir>/<classDir>/<stem>.txt,
// returning nil when the file is absent or unreadable.
func Load(labelsDir, classDir, stem string) []Record {
	path := filepath.Join(labelsDir, classDir, stem+".txt")
	f, err := os.Open(path)
	if err != nil {
		return nil
	}
	defer f.Close()

	var records []Record
	sc := bufio.NewScanner(f)
	for sc.Scan() {
		if r, ok := parseLine(sc.Text()); ok {
			records = append(records, r)
		}
	}
	return records
}

func parseLine(line string) (Record, bool) {
	fields := strings.Fields(line)
	if len(fields) == 0 {
		return Record{}, false
	}
	classID, err := strconv.Atoi(fields[0])
	if err != nil {
		return Record{}, false
	}
	coords := make([]float64, 0, len(fields)-1)
	for _, field := range fields[1:] {
		v, err := strconv.ParseFloat(field, 64)
		if err != nil {
			return Record{}, false
		}
		coords = append(coords, v)
	}
	return Record{ClassID: classID, Coords: coords}, true
}

// PolygonFromCoords converts a flat [x1, y1, x2, y2, ...] list into clamped points.
func PolygonFromCoords(coords []float64) []Point {
	points := make([]Point, 0, len(coords)/2)
	for i := 0; i+1 < len(coords); i += 2 {
		points = append(points, Point{X: clamp01(coords[i]), Y: clamp01(coords[i+1])})
	}
	return points
}

// CornersFromBBox converts YOLO [xc, yc, w, h] into four clamped corner points
// ordered top-left, top-right, bottom-right, bottom-left.
func CornersFromBBox(xc, yc, w, h float64) []Point {
	halfW, halfH := w/2, h/2
	return []Point{
		{X: clamp01(xc - halfW), Y: clamp01(yc - halfH)},
		{X: clamp01(xc + halfW), Y: clamp01(yc - halfH)},
		{X: clamp01(xc + halfW), Y: clamp01(yc + halfH)},
		{X: clamp01(xc - halfW), Y: clamp01(yc + halfH)},
	}
}

func clamp01(v float64) float64 {
	if v < 0 {
		return 0
	}
	if v > 1 {
		return 1
	}
	return v
}
