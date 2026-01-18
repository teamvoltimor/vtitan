package main

import (
	"bufio"
	"fmt"
	"io/fs"
	"os"
	"path/filepath"
	"strings"
	"time"
)

// Source and destination directories (relative to project root)
const (
	docsDir    = "../../docs"
	contentDir = "../content/v1"
)

// Checks if a file has Hugo front matter (YAML or TOML)
func hasFrontMatter(path string) (bool, error) {
	f, err := os.Open(path)
	if err != nil {
		return false, err
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)
	if scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		return line == "---" || line == "+++", nil
	}
	return false, nil
}

// Extracts the first Markdown heading as the title, or uses the filename
func extractTitle(path string, fallback string) (string, error) {
	f, err := os.Open(path)
	if err != nil {
		return fallback, err
	}
	defer f.Close()

	scanner := bufio.NewScanner(f)
	for scanner.Scan() {
		line := strings.TrimSpace(scanner.Text())
		if strings.HasPrefix(line, "#") {
			return strings.TrimSpace(strings.TrimLeft(line, "#")), nil
		}
	}
	return fallback, nil
}

// Adds Hugo YAML front matter to a Markdown file's content
func addFrontMatter(original []byte, title string) []byte {
	date := time.Now().Format("2006-01-02")
	frontMatter := fmt.Sprintf("---\ntitle: \"%s\"\ndate: %s\n---\n\n", title, date)
	return append([]byte(frontMatter), original...)
}

// Copies a single Markdown file, adding front matter if missing
func processFile(srcPath, dstPath string) error {
	// Ensure destination directory exists
	if err := os.MkdirAll(filepath.Dir(dstPath), 0755); err != nil {
		return err
	}

	hasFM, err := hasFrontMatter(srcPath)
	if err != nil {
		return err
	}

	input, err := os.ReadFile(srcPath)
	if err != nil {
		return err
	}

	if hasFM {
		// Copy as-is
		return os.WriteFile(dstPath, input, 0644)
	}

	// Add front matter
	base := filepath.Base(srcPath)
	name := strings.TrimSuffix(base, filepath.Ext(base))
	title, _ := extractTitle(srcPath, name)
	output := addFrontMatter(input, title)
	return os.WriteFile(dstPath, output, 0644)
}

// Recursively copies all .md files from docsDir to contentDir, preserving structure
func copyDocs() error {
	return filepath.WalkDir(docsDir, func(path string, d fs.DirEntry, err error) error {
		if err != nil {
			return err
		}
		if d.IsDir() {
			return nil
		}
		if strings.HasSuffix(strings.ToLower(d.Name()), ".md") {
			rel, err := filepath.Rel(docsDir, path)
			if err != nil {
				return err
			}
			dstPath := filepath.Join(contentDir, rel)
			fmt.Printf("Processing %s -> %s\n", path, dstPath)
			return processFile(path, dstPath)
		}
		return nil
	})
}

func main() {
	fmt.Println("Copying Markdown files from", docsDir, "to", contentDir)
	if err := copyDocs(); err != nil {
		fmt.Fprintf(os.Stderr, "Error: %v\n", err)
		os.Exit(1)
	}
	fmt.Println("Done.")
}
