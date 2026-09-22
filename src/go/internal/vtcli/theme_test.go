package vtcli

import (
	"os"
	"path/filepath"
	"regexp"
	"strings"
	"testing"
)

// TestPaletteMatchesDocsTokens keeps vt on the project's colour system: each
// palette entry must equal its docs token, light value from :root and dark
// value from html.dark, in the order the stylesheet declares them.
func TestPaletteMatchesDocsTokens(t *testing.T) {
	t.Parallel()

	root, err := FindRepoRoot(".")
	if err != nil {
		t.Fatalf("find repo root: %v", err)
	}

	raw, err := os.ReadFile(filepath.Join(root, docsTokensFile))
	if err != nil {
		t.Fatalf("read %s: %v", docsTokensFile, err)
	}

	for token, colour := range paletteTokens {
		pattern := regexp.MustCompile(`--color-accent-` + token + `:\s*(#[0-9a-fA-F]{6})\s*;`)
		matches := pattern.FindAllStringSubmatch(string(raw), -1)

		if len(matches) != 2 {
			t.Errorf("--color-accent-%s: want a light and a dark value, found %d", token, len(matches))

			continue
		}

		light, dark := strings.ToLower(matches[0][1]), strings.ToLower(matches[1][1])
		if colour.Light != light || colour.Dark != dark {
			t.Errorf("--color-accent-%s is %s/%s in %s, vt has %s/%s",
				token, light, dark, docsTokensFile, colour.Light, colour.Dark)
		}
	}
}
