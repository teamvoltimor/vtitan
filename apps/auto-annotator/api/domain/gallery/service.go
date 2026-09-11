package gallery

import (
	"context"
	"database/sql"
	"errors"
	"fmt"
	"image"
	"image/jpeg"

	// Registers the PNG decoder with image.Decode; imports for uploaded images may
	// be PNG even though generated thumbnails are always re-encoded as JPEG.
	_ "image/png"
	"io"
	"log/slog"
	"math"
	"mime/multipart"
	"os"
	"path/filepath"
	"strconv"

	"golang.org/x/image/draw"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/annotation"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/config"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

type galleryService struct {
	store Store
	cfg   config.Config
}

// NewService returns a gallery Service backed by the given store and config.
func NewService(store Store, cfg config.Config) Service {
	return &galleryService{store: store, cfg: cfg}
}

func (s *galleryService) GetGallery(ctx context.Context) (Gallery, error) {
	return s.buildGallery(ctx)
}

func (s *galleryService) GetGroupedGallery(ctx context.Context) ([]GroupedImage, error) {
	rows, err := s.store.ListGroupedImages(ctx)
	if err != nil {
		return nil, err
	}
	for i := range rows {
		rows[i].ImageURL = s.imageURL(rows[i].ID)
		rows[i].Label = filepath.Base(rows[i].Path)
	}
	return rows, nil
}

func (s *galleryService) Import(ctx context.Context, files []*multipart.FileHeader) (Gallery, error) {
	if err := ensureDir(s.pendingDir()); err != nil {
		return Gallery{}, err
	}
	for _, fh := range files {
		safe := filepath.Base(fh.Filename)
		dest := filepath.Join(s.pendingDir(), randomHex()+domain.UnderscoreSep+safe)
		src, err := fh.Open()
		if err != nil {
			return Gallery{}, fmt.Errorf("open upload %q: %w", safe, err)
		}
		saveErr := saveReader(src, dest)
		src.Close()
		if saveErr != nil {
			return Gallery{}, fmt.Errorf("save upload %q: %w", safe, saveErr)
		}
		if err := s.store.InsertImageOrIgnore(ctx, dest); err != nil {
			return Gallery{}, err
		}
	}
	return s.buildGallery(ctx)
}

func (s *galleryService) Delete(ctx context.Context, ids []int64) (Gallery, error) {
	for _, id := range ids {
		img, err := s.store.GetImage(ctx, id)
		if errors.Is(err, sql.ErrNoRows) {
			return Gallery{}, fmt.Errorf("image %d: %w", id, domain.ErrNotFound)
		}
		if err != nil {
			return Gallery{}, err
		}
		if err := s.store.SoftDeleteImage(ctx, id); err != nil {
			return Gallery{}, err
		}
		_ = os.Remove(img.Path)
	}
	return s.buildGallery(ctx)
}

func (s *galleryService) ImagePath(ctx context.Context, id int64) (string, error) {
	img, err := s.store.GetImage(ctx, id)
	if err != nil {
		return "", err
	}
	return img.Path, nil
}

func (s *galleryService) ThumbPath(ctx context.Context, id int64) (string, error) {
	img, err := s.store.GetImage(ctx, id)
	if err != nil {
		return "", err
	}
	thumbPath := filepath.Join(s.thumbsDir(), strconv.FormatInt(id, 10)+domain.ThumbFileExt)
	if thumbIsFresh(thumbPath, img.Path) {
		return thumbPath, nil
	}
	if err := s.generateThumbnail(img.Path, thumbPath); err != nil {
		// Falling back to the original image is intentional (still lets the
		// gallery render), but a systematically broken thumbnailer should be
		// visible in the logs rather than silent.
		slog.Warn("thumbnail generation failed, falling back to original image",
			"image_id", id, "image_path", img.Path, "error", err)
		return img.Path, nil
	}
	return thumbPath, nil
}

func (s *galleryService) buildGallery(ctx context.Context) (Gallery, error) {
	rows, err := s.store.ListImagesForBrowse(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", domain.ErrCtxListImages, err)
	}
	counts, err := s.store.GetStatusCounts(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", domain.ErrCtxStatusCounts, err)
	}
	classNames, err := s.store.ListClassNames(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", domain.ErrCtxListClasses, err)
	}

	items := make([]GalleryItem, 0, len(rows))
	for _, row := range rows {
		status := domain.StatusName(row.Status)
		anns := annotation.LoadAnnotations(row.ID, row.Path, status, row.FormatUsed, classNames, s.labelsDir())
		items = append(items, GalleryItem{
			ID:          row.ID,
			Path:        row.Path,
			ImageURL:    s.imageURL(row.ID),
			ThumbURL:    s.thumbURL(row.ID),
			Format:      row.FormatUsed,
			Status:      status,
			UpdatedAt:   row.UpdatedAt,
			Annotations: anns,
		})
	}
	return Gallery{Items: items, Stats: computeStats(counts)}, nil
}

func (s *galleryService) generateThumbnail(srcPath, thumbPath string) error {
	f, err := os.Open(srcPath)
	if err != nil {
		return err
	}
	defer f.Close()

	src, _, err := image.Decode(f)
	if err != nil {
		return err
	}

	b := src.Bounds()
	w, h := b.Dx(), b.Dy()
	if w > domain.ThumbMaxWidth {
		h = h * domain.ThumbMaxWidth / w
		w = domain.ThumbMaxWidth
	}
	dst := image.NewRGBA(image.Rect(0, 0, w, h))
	draw.CatmullRom.Scale(dst, dst.Bounds(), src, b, draw.Over, nil)

	if err := ensureDir(s.thumbsDir()); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(s.thumbsDir(), domain.ThumbTempPattern)
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	if err := jpeg.Encode(tmp, dst, &jpeg.Options{Quality: domain.ThumbJPEGQuality}); err != nil {
		tmp.Close()
		os.Remove(tmpName)
		return err
	}
	if err := tmp.Close(); err != nil {
		os.Remove(tmpName)
		return err
	}
	return os.Rename(tmpName, thumbPath)
}

func (s *galleryService) imageURL(id int64) string {
	if s.cfg.APIPublicURL == "" {
		return fmt.Sprintf(domain.RelativeImageURLTemplate, id)
	}
	return fmt.Sprintf(domain.ImageURLTemplate, s.cfg.APIPublicURL, id)
}

func (s *galleryService) thumbURL(id int64) string { return s.imageURL(id) + "/thumb" }

func (s *galleryService) labelsDir() string {
	return filepath.Join(s.cfg.DataDir, domain.LabelsDirName)
}
func (s *galleryService) pendingDir() string {
	return filepath.Join(s.cfg.DataDir, domain.PendingDirName)
}
func (s *galleryService) thumbsDir() string {
	return filepath.Join(s.cfg.DataDir, domain.ThumbsDirName)
}

func computeStats(counts []StatusCount) Stats {
	var pending, done, skipped int64
	for _, c := range counts {
		switch c.Status {
		case domain.StatusPending:
			pending = c.Count
		case domain.StatusDone:
			done = c.Count
		case domain.StatusSkipped:
			skipped = c.Count
		}
	}
	total := pending + done + skipped
	var pct float64
	if total > 0 {
		pct = math.Round(float64(done)/float64(total)*domain.PercentageScale) / domain.PercentageDivisor
	}
	return Stats{
		Pending: int(pending),
		Done:    int(done),
		Skipped: int(skipped),
		Total:   int(total),
		Pct:     pct,
	}
}

func thumbIsFresh(thumbPath, srcPath string) bool {
	ti, err := os.Stat(thumbPath)
	if err != nil {
		return false
	}
	si, err := os.Stat(srcPath)
	if err != nil {
		return false
	}
	return !ti.ModTime().Before(si.ModTime())
}

func randomHex() string {
	return domain.RandomHexString(domain.FallbackHexString)
}

func saveReader(src io.Reader, dest string) error {
	out, err := os.Create(dest)
	if err != nil {
		return err
	}
	if _, err := io.Copy(out, src); err != nil {
		out.Close()
		return err
	}
	return out.Close()
}

func ensureDir(path string) error {
	return os.MkdirAll(path, domain.DirPerm)
}
