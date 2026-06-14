package handlers

import (
	"context"
	"database/sql"
	"errors"
	"fmt"
	"io"
	"net/http"
	"os"
	"path/filepath"
	"strings"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/dataset"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/domain"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/labels"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store/db"
)

// SaveAnnotations writes the label file + image copy under per-class
// directories, marks the image done, regenerates data.yaml, and returns the
// updated gallery. POST /annotations/save
func (a *App) SaveAnnotations(c *gin.Context) {
	var req dto.SaveAnnotationsRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}

	ctx := c.Request.Context()
	rec, err := a.Store.Q.GetImageByID(ctx, req.ImageID)
	if errors.Is(err, sql.ErrNoRows) {
		problem.Write(c, http.StatusNotFound, "Image not found", "")
		return
	}
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}

	classes, err := a.Store.Q.ListClasses(ctx)
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	if msg := validateSave(req, classes); msg != "" {
		problem.Write(c, http.StatusBadRequest, msg, "")
		return
	}

	// YOLO class index is the position in the id-ordered class list.
	nameToYOLO := make(map[string]int, len(classes))
	for i, cls := range classes {
		nameToYOLO[cls.Name] = i
	}
	exportFmt := FormatDetectionAbbr
	if req.ExportFormat == ExportFormatSegmentation {
		exportFmt = FormatSegmentationAbbr
	}

	records := make([]labels.Record, 0, len(req.Shapes))
	for _, shape := range req.Shapes {
		idx, ok := nameToYOLO[shape.ClassName]
		if !ok {
			continue
		}
		var coords []float64
		if exportFmt == FormatSegmentationAbbr {
			coords = make([]float64, 0, len(shape.Points)*2)
			for _, p := range shape.Points {
				coords = append(coords, p.X, p.Y)
			}
		} else {
			coords = labels.BBoxFromPolygon(toLabelPoints(shape.Points))
		}
		records = append(records, labels.Record{ClassID: idx, Coords: coords})
	}

	primaryClass := req.Shapes[0].ClassName
	if markErr := a.markDone(ctx, rec, primaryClass, exportFmt, records, classes); markErr != nil {
		problem.Write(c, http.StatusInternalServerError, markErr.Error(), "")
		return
	}

	resp, err := a.buildGalleryResponse(ctx)
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusCreated, resp)
}

// SkipImage marks an image as skipped and returns the updated gallery.
// POST /annotations/skip
func (a *App) SkipImage(c *gin.Context) {
	var req dto.SkipRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}

	ctx := c.Request.Context()
	if _, err := a.Store.Q.GetImageByID(ctx, req.ImageID); errors.Is(err, sql.ErrNoRows) {
		problem.Write(c, http.StatusNotFound, "Image not found", "")
		return
	} else if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}

	if err := a.Store.Q.MarkImageSkipped(ctx, db.MarkImageSkippedParams{
		Status:    domain.StatusSkipped,
		UpdatedAt: sql.NullString{String: time.Now().UTC().Format(TimeFormatISO8601), Valid: true},
		ID:        req.ImageID,
	}); err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}

	resp, err := a.buildGalleryResponse(ctx)
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, resp)
}

// markDone copies the source image and writes its label under per-class
// directories, updates DB status (the commit point), then regenerates data.yaml.
func (a *App) markDone(
	ctx context.Context,
	rec db.GetImageByIDRow,
	primaryClass, exportFmt string,
	records []labels.Record,
	classes []db.ListClassesRow,
) error {
	imgDestDir := filepath.Join(a.imagesDir(), primaryClass)
	lblDestDir := filepath.Join(a.labelsDir(), primaryClass)
	if err := ensureDir(imgDestDir); err != nil {
		return err
	}
	if err := ensureDir(lblDestDir); err != nil {
		return err
	}

	base := filepath.Base(rec.Path)
	if err := copyFile(rec.Path, filepath.Join(imgDestDir, base)); err != nil {
		return fmt.Errorf("copy image: %w", err)
	}
	stem := strings.TrimSuffix(base, filepath.Ext(base))
	if err := labels.Write(filepath.Join(lblDestDir, stem+".txt"), records); err != nil {
		return fmt.Errorf("write label: %w", err)
	}

	if err := a.Store.Q.MarkImageDone(ctx, db.MarkImageDoneParams{
		Status:     domain.StatusDone,
		FormatUsed: sql.NullString{String: exportFmt, Valid: true},
		UpdatedAt:  sql.NullString{String: time.Now().UTC().Format(TimeFormatISO8601), Valid: true},
		ID:         rec.ID,
	}); err != nil {
		return fmt.Errorf("mark done: %w", err)
	}

	classNames := make([]string, len(classes))
	for i, cls := range classes {
		classNames[i] = cls.Name
	}
	return dataset.WriteDataYAML(a.Cfg.DataDir, classNames)
}

// validateSave mirrors DefaultValidator.validate_save_annotations_request,
// returning a non-empty message on failure.
func validateSave(req dto.SaveAnnotationsRequest, classes []db.ListClassesRow) string {
	if len(req.Shapes) == 0 {
		return "At least one shape required"
	}
	known := make(map[string]bool, len(classes))
	for _, cls := range classes {
		known[cls.Name] = true
	}
	if primary := req.Shapes[0].ClassName; !known[primary] {
		return fmt.Sprintf("Unknown primary class '%s'", primary)
	}
	for _, shape := range req.Shapes {
		if !known[shape.ClassName] {
			return fmt.Sprintf("Unknown class '%s'", shape.ClassName)
		}
	}
	return ""
}

func toLabelPoints(pts []dto.Point) []labels.Point {
	out := make([]labels.Point, len(pts))
	for i, p := range pts {
		out[i] = labels.Point{X: p.X, Y: p.Y}
	}
	return out
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
