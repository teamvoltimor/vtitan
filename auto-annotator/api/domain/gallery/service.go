package gallery

import (
	"context"
	"crypto/rand"
	"database/sql"
	"encoding/hex"
	"errors"
	"fmt"
	"image"
	"image/jpeg"
	_ "image/png"
	"io"
	"math"
	"mime/multipart"
	"os"
	"path/filepath"
	"strconv"

	"golang.org/x/image/draw"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/domain"
)

const (
	pendingDirName    = "pending"
	thumbsDirName     = "thumbs"
	labelsDirName     = "labels"
	thumbMaxWidth     = 320
	thumbJPEGQuality  = 80
	thumbFileExt      = ".jpg"
	thumbTempPattern  = "thumb-*.jpg"
	underscoreSep     = "_"
	fallbackHex       = "00000000000000000000000000000000"
	randomHexBytes    = 16
	percentageScale   = 1000
	percentageDivisor = 10
	dirPerm           = 0o750

	relativeImageURLTemplate = "/api/v1/images/%d"
	imageURLTemplate         = "%s/api/v1/images/%d"

	errCtxListImages   = "list images"
	errCtxStatusCounts = "status counts"
	errCtxListClasses  = "list classes"
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
		dest := filepath.Join(s.pendingDir(), randomHex()+underscoreSep+safe)
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
	thumbPath := filepath.Join(s.thumbsDir(), strconv.FormatInt(id, 10)+thumbFileExt)
	if thumbIsFresh(thumbPath, img.Path) {
		return thumbPath, nil
	}
	if err := s.generateThumbnail(img.Path, thumbPath); err != nil {
		return img.Path, nil
	}
	return thumbPath, nil
}

func (s *galleryService) buildGallery(ctx context.Context) (Gallery, error) {
	rows, err := s.store.ListImagesForBrowse(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", errCtxListImages, err)
	}
	counts, err := s.store.GetStatusCounts(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", errCtxStatusCounts, err)
	}
	classNames, err := s.store.ListClassNames(ctx)
	if err != nil {
		return Gallery{}, fmt.Errorf("%s: %w", errCtxListClasses, err)
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
	if w > thumbMaxWidth {
		h = h * thumbMaxWidth / w
		w = thumbMaxWidth
	}
	dst := image.NewRGBA(image.Rect(0, 0, w, h))
	draw.CatmullRom.Scale(dst, dst.Bounds(), src, b, draw.Over, nil)

	if err := ensureDir(s.thumbsDir()); err != nil {
		return err
	}
	tmp, err := os.CreateTemp(s.thumbsDir(), thumbTempPattern)
	if err != nil {
		return err
	}
	tmpName := tmp.Name()
	if err := jpeg.Encode(tmp, dst, &jpeg.Options{Quality: thumbJPEGQuality}); err != nil {
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
		return fmt.Sprintf(relativeImageURLTemplate, id)
	}
	return fmt.Sprintf(imageURLTemplate, s.cfg.APIPublicURL, id)
}

func (s *galleryService) thumbURL(id int64) string { return s.imageURL(id) + "/thumb" }

func (s *galleryService) labelsDir() string  { return filepath.Join(s.cfg.DataDir, labelsDirName) }
func (s *galleryService) pendingDir() string { return filepath.Join(s.cfg.DataDir, pendingDirName) }
func (s *galleryService) thumbsDir() string  { return filepath.Join(s.cfg.DataDir, thumbsDirName) }

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
		pct = math.Round(float64(done)/float64(total)*percentageScale) / percentageDivisor
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
	var b [randomHexBytes]byte
	if _, err := rand.Read(b[:]); err != nil {
		return fallbackHex
	}
	return hex.EncodeToString(b[:])
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
	return os.MkdirAll(path, dirPerm)
}
