package annotation

import (
	"context"
	"database/sql"
	"errors"
	"fmt"
	"io"
	"os"
	"path/filepath"
	"strings"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/dataset"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/config"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

type annotationService struct {
	store Store
	cfg   config.Config
}

// NewService returns an annotation Service backed by the given store and config.
func NewService(store Store, cfg config.Config) Service {
	return &annotationService{store: store, cfg: cfg}
}

func (s *annotationService) GetAnnotations(ctx context.Context, imageID int64) ([]Shape, error) {
	img, err := s.store.GetImage(ctx, imageID)
	if err != nil {
		return nil, err
	}
	classes, err := s.store.ListClasses(ctx)
	if err != nil {
		return nil, err
	}
	names := make([]string, len(classes))
	for i, c := range classes {
		names[i] = c.Name
	}
	shapes := LoadAnnotations(img.ID, img.Path, domain.StatusName(img.Status), img.FormatUsed, names, s.labelsDir())
	if shapes == nil {
		shapes = []Shape{}
	}
	return shapes, nil
}

func (s *annotationService) Save(ctx context.Context, req SaveRequest) error {
	img, err := s.store.GetImage(ctx, req.ImageID)
	if err != nil {
		if errors.Is(err, sql.ErrNoRows) {
			return fmt.Errorf("image %d: %w", req.ImageID, domain.ErrNotFound)
		}
		return err
	}
	classes, err := s.store.ListClasses(ctx)
	if err != nil {
		return err
	}
	if msg := validateSave(req, classes); msg != "" {
		return fmt.Errorf("%s: %w", msg, domain.ErrInvalidInput)
	}

	nameToYOLO := make(map[string]int, len(classes))
	for i, cls := range classes {
		nameToYOLO[cls.Name] = i
	}

	exportFmt := domain.FormatDetectionAbbr
	if req.ExportFormat == domain.ExportFormatSegmentation {
		exportFmt = domain.FormatSegmentationAbbr
	}

	records := make([]dataset.Record, 0, len(req.Shapes))
	for _, shape := range req.Shapes {
		idx, ok := nameToYOLO[shape.ClassName]
		if !ok {
			continue
		}
		var coords []float64
		if exportFmt == domain.FormatSegmentationAbbr {
			coords = make([]float64, 0, len(shape.Points)*2)
			for _, p := range shape.Points {
				coords = append(coords, p.X, p.Y)
			}
		} else {
			coords = dataset.BBoxFromPolygon(toDatasetPoints(shape.Points))
		}
		records = append(records, dataset.Record{ClassID: idx, Coords: coords})
	}

	primaryClass := req.Shapes[0].ClassName
	return s.markDone(ctx, img, primaryClass, exportFmt, records, classes)
}

func (s *annotationService) Skip(ctx context.Context, imageID int64) error {
	if _, err := s.store.GetImage(ctx, imageID); errors.Is(err, sql.ErrNoRows) {
		return fmt.Errorf("image %d: %w", imageID, domain.ErrNotFound)
	} else if err != nil {
		return err
	}
	return s.store.MarkImageSkipped(ctx, imageID)
}

func (s *annotationService) ListClasses(ctx context.Context) ([]Class, error) {
	return s.store.ListClasses(ctx)
}

func (s *annotationService) UpsertClass(ctx context.Context, req UpsertClassReq) error {
	return s.store.UpsertClass(ctx, req.Name, req.Color)
}

func (s *annotationService) markDone(
	ctx context.Context,
	img Image,
	primaryClass, exportFmt string,
	records []dataset.Record,
	classes []Class,
) error {
	imgDestDir := filepath.Join(s.imagesDir(), primaryClass)
	lblDestDir := filepath.Join(s.labelsDir(), primaryClass)
	if err := ensureDir(imgDestDir); err != nil {
		return err
	}
	if err := ensureDir(lblDestDir); err != nil {
		return err
	}

	base := filepath.Base(img.Path)
	if err := copyFile(img.Path, filepath.Join(imgDestDir, base)); err != nil {
		return fmt.Errorf("copy image: %w", err)
	}
	stem := strings.TrimSuffix(base, filepath.Ext(base))
	if err := dataset.Write(filepath.Join(lblDestDir, stem+domain.LabelFileExt), records); err != nil {
		return fmt.Errorf("write label: %w", err)
	}

	if err := s.store.MarkImageDone(ctx, img.ID, exportFmt); err != nil {
		return fmt.Errorf("mark done: %w", err)
	}

	classNames := make([]string, len(classes))
	for i, cls := range classes {
		classNames[i] = cls.Name
	}
	return dataset.WriteDataYAML(s.cfg.DataDir, classNames)
}

func (s *annotationService) labelsDir() string {
	return filepath.Join(s.cfg.DataDir, domain.LabelsDirName)
}
func (s *annotationService) imagesDir() string {
	return filepath.Join(s.cfg.DataDir, domain.ImagesDirName)
}

func validateSave(req SaveRequest, classes []Class) string {
	if len(req.Shapes) == 0 {
		return domain.ErrAtLeastOneShape
	}
	known := make(map[string]bool, len(classes))
	for _, cls := range classes {
		known[cls.Name] = true
	}
	if primary := req.Shapes[0].ClassName; !known[primary] {
		return fmt.Sprintf(domain.ErrFmtUnknownPrimaryClass, primary)
	}
	for _, shape := range req.Shapes {
		if !known[shape.ClassName] {
			return fmt.Sprintf(domain.ErrFmtUnknownClass, shape.ClassName)
		}
	}
	return ""
}

func toDatasetPoints(pts []Point) []dataset.Point {
	out := make([]dataset.Point, len(pts))
	for i, p := range pts {
		out[i] = dataset.Point{X: p.X, Y: p.Y}
	}
	return out
}

func ensureDir(path string) error {
	return os.MkdirAll(path, domain.DirPerm)
}

func copyFile(src, dst string) error {
	in, err := os.Open(src)
	if err != nil {
		return err
	}
	defer in.Close()
	out, err := os.Create(dst)
	if err != nil {
		return err
	}
	if _, err := io.Copy(out, in); err != nil {
		out.Close()
		return err
	}
	return out.Close()
}
