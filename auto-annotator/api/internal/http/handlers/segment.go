package handlers

import (
	"database/sql"
	"errors"
	"fmt"
	"net/http"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/compute"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store/db"
)

// Segment runs SAM inference for the given click points and returns the best
// mask as a polygon. POST /segment
func (a *App) Segment(c *gin.Context) {
	var req dto.SegmentationRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}

	ctx := c.Request.Context()
	rec, err := a.Store.Q.GetImageByID(ctx, req.ImageID)
	if errors.Is(err, sql.ErrNoRows) {
		problem.Write(c, http.StatusBadRequest, fmt.Sprintf("Image %d not found", req.ImageID), "")
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
	if msg := validateSegment(req, classes); msg != "" {
		problem.Write(c, http.StatusBadRequest, msg, "")
		return
	}

	classNames := make([]string, len(classes))
	for i, cls := range classes {
		classNames[i] = cls.Name
	}
	points := make([]compute.ClickPoint, len(req.Points))
	for i, p := range req.Points {
		points[i] = compute.ClickPoint{X: p.X, Y: p.Y, PointType: p.PointType, ClassName: p.ClassName}
	}

	res, err := a.Compute.Segment(ctx, compute.SegmentInput{
		ImageID:    rec.ID,
		ImagePath:  rec.Path,
		Points:     points,
		ClassNames: classNames,
	})
	if err != nil {
		problem.Write(c, http.StatusInternalServerError, err.Error(), "")
		return
	}

	c.JSON(http.StatusOK, dto.SegmentationResponse{
		State:   res.State,
		Message: res.Message,
		Shapes:  computeShapesToDTO(res.Shapes),
	})
}

// validateSegment mirrors DefaultValidator.validate_segmentation_request.
func validateSegment(req dto.SegmentationRequest, classes []db.ListClassesRow) string {
	if len(req.Points) == 0 {
		return "At least one point required"
	}
	known := make(map[string]bool, len(classes))
	for _, cls := range classes {
		known[cls.Name] = true
	}
	for _, p := range req.Points {
		if !known[p.ClassName] {
			return fmt.Sprintf("Unknown class '%s'", p.ClassName)
		}
	}
	return ""
}

func computeShapesToDTO(shapes []compute.Shape) []dto.Shape {
	out := make([]dto.Shape, 0, len(shapes))
	for _, s := range shapes {
		pts := make([]dto.Point, 0, len(s.Points))
		for _, p := range s.Points {
			pts = append(pts, dto.Point{X: p.X, Y: p.Y})
		}
		out = append(out, dto.Shape{ID: s.ID, ClassName: s.ClassName, Points: pts})
	}
	return out
}
