package gallery

import (
	"testing"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
)

func TestImageURL(t *testing.T) {
	t.Run("relative when no public url", func(t *testing.T) {
		svc := &galleryService{cfg: config.Config{APIPublicURL: ""}}
		if got := svc.imageURL(5); got != "/api/v1/images/5" {
			t.Fatalf("imageURL = %q, want /api/v1/images/5", got)
		}
		if got := svc.thumbURL(5); got != "/api/v1/images/5/thumb" {
			t.Fatalf("thumbURL = %q, want /api/v1/images/5/thumb", got)
		}
	})
	t.Run("absolute when public url set", func(t *testing.T) {
		svc := &galleryService{cfg: config.Config{APIPublicURL: "https://cdn.example.com"}}
		if got := svc.imageURL(7); got != "https://cdn.example.com/api/v1/images/7" {
			t.Fatalf("imageURL = %q", got)
		}
	})
}
