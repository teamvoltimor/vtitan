package dataset

import (
	"fmt"
	"math"
	"os"
	"path/filepath"
	"sort"
	"strings"
)

const (
	valFraction   = 0.2
	trainListName = "train.txt"
	valListName   = "val.txt"
	dataYAMLName  = "data.yaml"
)

var validExts = map[string]bool{
	".jpg": true, ".jpeg": true, ".png": true, ".bmp": true, ".webp": true,
}

// WriteDataYAML writes <dataDir>/{train.txt,val.txt,data.yaml} for the given classes.
func WriteDataYAML(dataDir string, classNames []string) error {
	if len(classNames) == 0 {
		return nil
	}
	imagesDir := filepath.Join(dataDir, "images")

	existing := make([]string, 0, len(classNames))
	for _, name := range classNames {
		if info, err := os.Stat(filepath.Join(imagesDir, name)); err == nil && info.IsDir() {
			existing = append(existing, name)
		}
	}
	if len(existing) == 0 {
		return nil
	}

	train, val := splitDataset(imagesDir, existing)
	if len(train) == 0 {
		return nil
	}

	if err := os.WriteFile(
		filepath.Join(dataDir, trainListName),
		[]byte(strings.Join(train, "\n")+"\n"),
		0o644,
	); err != nil {
		return fmt.Errorf("write train list: %w", err)
	}
	if err := os.WriteFile(
		filepath.Join(dataDir, valListName),
		[]byte(strings.Join(val, "\n")+"\n"),
		0o644,
	); err != nil {
		return fmt.Errorf("write val list: %w", err)
	}

	absDir, err := filepath.Abs(dataDir)
	if err != nil {
		absDir = dataDir
	}
	var b strings.Builder
	fmt.Fprintf(&b, "path: %s\n", absDir)
	fmt.Fprintf(&b, "train: %s\n", trainListName)
	fmt.Fprintf(&b, "val: %s\n\n", valListName)
	fmt.Fprintf(&b, "nc: %d\n", len(classNames))
	b.WriteString("names:\n")
	for _, name := range classNames {
		fmt.Fprintf(&b, "  - %s\n", name)
	}

	if err := os.WriteFile(filepath.Join(dataDir, dataYAMLName), []byte(b.String()), 0o644); err != nil {
		return fmt.Errorf("write data.yaml: %w", err)
	}
	return nil
}

func splitDataset(imagesDir string, classNames []string) (train, val []string) {
	for _, name := range classNames {
		entries, err := os.ReadDir(filepath.Join(imagesDir, name))
		if err != nil {
			continue
		}
		files := make([]string, 0, len(entries))
		for _, e := range entries {
			if e.IsDir() {
				continue
			}
			if validExts[strings.ToLower(filepath.Ext(e.Name()))] {
				files = append(files, e.Name())
			}
		}
		if len(files) == 0 {
			continue
		}
		sort.Strings(files)

		nVal := 0
		if len(files) >= 2 {
			nVal = max(1, int(math.Round(float64(len(files))*valFraction)))
		}
		for _, f := range files[:nVal] {
			val = append(val, fmt.Sprintf("images/%s/%s", name, f))
		}
		for _, f := range files[nVal:] {
			train = append(train, fmt.Sprintf("images/%s/%s", name, f))
		}
	}

	if len(val) == 0 && len(train) >= 2 {
		val = append(val, train[len(train)-1])
		train = train[:len(train)-1]
	}
	return train, val
}
