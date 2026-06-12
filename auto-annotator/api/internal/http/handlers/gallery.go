package handlers

import (
	"context"
	"crypto/rand"
	"encoding/hex"
	"fmt"
	"math"
	"net/http"
	"path/filepath"
	"strings"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/domain"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/labels"
	"github.com/teamvoldemor/voldemorbot-auto-annotator/api/internal/store/db"
)

const (
	formatDetection = "det"
)

// GetGallery returns all images with status counts. GET /gallery
func (a *App) GetGallery(c *gin.Context) {
	resp, err := a.buildGalleryResponse(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, resp)
}

// GetGroupedGallery returns parent images with augmentation counts.
// GET /gallery/grouped
func (a *App) GetGroupedGallery(c *gin.Context) {
	rows, err := a.Store.Q.ListGroupedImages(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	items := make([]dto.ParentImageItem, 0, len(rows))
	for _, r := range rows {
		items = append(items, dto.ParentImageItem{
			ID:        r.ID,
			Label:     filepath.Base(r.Path),
			Src:       a.imageURL(r.ID),
			Format:    nullStr(r.FormatUsed),
			Status:    domain.StatusName(r.Status),
			UpdatedAt: nullStr(r.UpdatedAt),
			AugCount:  r.AugCount,
		})
	}
	c.JSON(http.StatusOK, items)
}

// ImportGallery accepts uploaded image files, stages them in pending/, and
// returns the updated gallery. POST /gallery/import
func (a *App) ImportGallery(c *gin.Context) {
	form, err := c.MultipartForm()
	if err != nil {
		problem.Write(c, http.StatusBadRequest, err.Error(), "")
		return
	}
	files := form.File["files"]
	if len(files) == 0 {
		problem.Write(c, http.StatusBadRequest, "No files provided", "")
		return
	}
	if err := ensureDir(a.pendingDir()); err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}

	for _, fh := range files {
		safe := filepath.Base(fh.Filename)
		dest := filepath.Join(a.pendingDir(), randomHex()+"_"+safe)
		if err := c.SaveUploadedFile(fh, dest); err != nil {
			problem.Write(c, http.StatusInternalServerError, err.Error(), "")
			return
		}
		if err := a.Store.Q.InsertImageOrIgnore(c.Request.Context(), dest); err != nil {
			problem.Write(c, http.StatusInternalServerError, err.Error(), "")
			return
		}
	}

	resp, err := a.buildGalleryResponse(c.Request.Context())
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}
	c.JSON(http.StatusOK, resp)
}

// buildGalleryResponse assembles the full gallery payload (items + stats),
// loading saved annotations for each done image.
func (a *App) buildGalleryResponse(ctx context.Context) (dto.GalleryResponse, error) {
	rows, err := a.Store.Q.ListImagesForBrowse(ctx)
	if err != nil {
		return dto.GalleryResponse{}, fmt.Errorf("list images: %w", err)
	}
	counts, err := a.Store.Q.GetStatusCounts(ctx)
	if err != nil {
		return dto.GalleryResponse{}, fmt.Errorf("status counts: %w", err)
	}
	classes, err := a.Store.Q.ListClasses(ctx)
	if err != nil {
		return dto.GalleryResponse{}, fmt.Errorf("list classes: %w", err)
	}
	classNames := make([]string, len(classes))
	for i, cls := range classes {
		classNames[i] = cls.Name
	}

	items := make([]dto.GalleryItem, 0, len(rows))
	for _, row := range rows {
		status := domain.StatusName(row.Status)
		format := nullStr(row.FormatUsed)
		items = append(items, dto.GalleryItem{
			ID:          row.ID,
			Label:       filepath.Base(row.Path),
			Src:         a.imageURL(row.ID),
			Format:      format,
			Status:      status,
			Updated:     nullStr(row.UpdatedAt),
			Annotations: a.loadAnnotations(row.ID, row.Path, status, format, classNames),
		})
	}

	return dto.GalleryResponse{Items: items, Stats: computeStats(counts)}, nil
}

// loadAnnotations reads and converts the saved YOLO labels for a done image.
func (a *App) loadAnnotations(imageID int64, imagePath, status, format string, classNames []string) []dto.Shape {
	if status != domain.StatusName(domain.StatusDone) || format == "" {
		return []dto.Shape{}
	}
	classDir := filepath.Base(filepath.Dir(imagePath))
	stem := strings.TrimSuffix(filepath.Base(imagePath), filepath.Ext(imagePath))

	records := labels.Load(a.labelsDir(), classDir, stem)
	shapes := make([]dto.Shape, 0, len(records))
	for i, rec := range records {
		name := fmt.Sprintf("class_%d", rec.ClassID)
		if rec.ClassID >= 0 && rec.ClassID < len(classNames) {
			name = classNames[rec.ClassID]
		}

		var pts []labels.Point
		switch {
		case format == formatDetection && len(rec.Coords) == 4:
			pts = labels.CornersFromBBox(rec.Coords[0], rec.Coords[1], rec.Coords[2], rec.Coords[3])
		case len(rec.Coords) >= 4 && len(rec.Coords)%2 == 0:
			pts = labels.PolygonFromCoords(rec.Coords)
		default:
			continue
		}

		shapes = append(shapes, dto.Shape{
			ID:        fmt.Sprintf("loaded-%d-%d", imageID, i),
			ClassName: name,
			Points:    toDTOPoints(pts),
		})
	}
	return shapes
}

func (a *App) imageURL(id int64) string {
	return fmt.Sprintf("%s/api/v1/images/%d", a.Cfg.APIPublicURL, id)
}

func computeStats(counts []db.GetStatusCountsRow) dto.GalleryStats {
	var pending, done, skipped int64
	for _, c := range counts {
		switch c.Status {
		case domain.StatusPending:
			pending = c.Cnt
		case domain.StatusDone:
			done = c.Cnt
		case domain.StatusSkipped:
			skipped = c.Cnt
		}
	}
	total := pending + done + skipped
	var pct float64
	if total > 0 {
		pct = math.Round(float64(done)/float64(total)*1000) / 10
	}
	return dto.GalleryStats{
		Pending: int(pending),
		Done:    int(done),
		Skipped: int(skipped),
		Total:   int(total),
		Pct:     pct,
	}
}

func toDTOPoints(pts []labels.Point) []dto.Point {
	out := make([]dto.Point, 0, len(pts))
	for _, p := range pts {
		out = append(out, dto.Point{X: p.X, Y: p.Y})
	}
	return out
}

func randomHex() string {
	var b [16]byte
	if _, err := rand.Read(b[:]); err != nil {
		return "00000000000000000000000000000000"
	}
	return hex.EncodeToString(b[:])
}
