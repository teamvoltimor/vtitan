package handlers

import (
	"context"
	"database/sql"
	"encoding/json"
	"errors"
	"fmt"
	"io"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/compute"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/domain"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/dto"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/jobs"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/store/db"
)

// JobStatus reports whether any job is running. GET /augment/status, /train/status
func (a *App) JobStatus(c *gin.Context) {
	c.JSON(http.StatusOK, dto.JobStatusResponse{Running: a.Jobs.Running()})
}

// StartAugment launches an augmentation job. POST /augment/start
func (a *App) StartAugment(c *gin.Context) {
	var req dto.AugmentRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}
	if len(req.ImageIDs) == 0 {
		problem.Write(c, http.StatusBadRequest, "No images specified", "")
		return
	}
	if a.Jobs.Running() {
		problem.Write(c, http.StatusConflict, "Augmentation already running", "")
		return
	}

	num := req.NumAugmentations
	if num <= 0 {
		num = DefaultNumAugmentations
	}

	// Resolve sources from the DB (the API is the single DB owner); missing ids
	// are skipped, matching the Python augment loop.
	ctx := c.Request.Context()
	sources := make([]compute.AugmentSource, 0, len(req.ImageIDs))
	for _, id := range req.ImageIDs {
		rec, err := a.Store.Q.GetImageByID(ctx, id)
		if errors.Is(err, sql.ErrNoRows) {
			continue
		}
		if err != nil {
			problem.Write(c, http.StatusInternalServerError, err.Error(), "")
			return
		}
		format := nullStr(rec.FormatUsed)
		if format == "" {
			format = FormatDetectionAbbr
		}
		sources = append(sources, compute.AugmentSource{ImageID: rec.ID, Path: rec.Path, FormatUsed: format})
	}

	in := compute.AugmentInput{Sources: sources, NumAugmentations: num}
	if _, err := a.Jobs.Start(func(ctx context.Context, emit jobs.EmitFunc) {
		a.runAugment(ctx, emit, in)
	}); err != nil {
		problem.Write(c, http.StatusConflict, "Augmentation already running", "")
		return
	}
	c.JSON(http.StatusAccepted, dto.JobStatusResponse{Running: true, Message: "Augmentation started"})
}

// StartTrain launches a YOLO training job. POST /train/start
func (a *App) StartTrain(c *gin.Context) {
	var req dto.TrainRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.Write(c, http.StatusUnprocessableEntity, err.Error(), "Validation Error")
		return
	}
	if a.Jobs.Running() {
		problem.Write(c, http.StatusConflict, "Training already running", "")
		return
	}

	in := compute.TrainInput{
		ModelName:    orStr(req.ModelName, DefaultTrainModel),
		Epochs:       orInt(req.Epochs, DefaultTrainEpochs),
		Batch:        orInt(req.Batch, DefaultTrainBatch),
		Imgsz:        orInt(req.Imgsz, DefaultTrainImgsz),
		DataYamlPath: a.dataYamlPath(),
	}
	if _, err := a.Jobs.Start(func(ctx context.Context, emit jobs.EmitFunc) {
		a.runTrain(ctx, emit, in)
	}); err != nil {
		problem.Write(c, http.StatusConflict, "Training already running", "")
		return
	}
	c.JSON(http.StatusAccepted, dto.JobStatusResponse{Running: true, Message: "Training started"})
}

// runAugment relays gRPC augmentation progress to SSE and persists each produced
// image (the API owns the DB).
func (a *App) runAugment(ctx context.Context, emit jobs.EmitFunc, in compute.AugmentInput) {
	err := a.Compute.RunAugmentation(ctx, in, func(p compute.Progress) {
		if p.Augmented != nil {
			a.insertAugmented(ctx, *p.Augmented)
		}
		if p.Error == "" {
			emit(jobs.StatusRunning, p.Message, progressData(p))
		}
	})
	emitTerminal(emit, "Augmentation completed", err)
}

// runTrain relays gRPC training progress to SSE.
func (a *App) runTrain(ctx context.Context, emit jobs.EmitFunc, in compute.TrainInput) {
	err := a.Compute.RunTraining(ctx, in, func(p compute.Progress) {
		if p.Error == "" {
			emit(jobs.StatusRunning, p.Message, progressData(p))
		}
	})
	emitTerminal(emit, "Training completed", err)
}

func (a *App) insertAugmented(ctx context.Context, img compute.AugmentedImage) {
	_ = a.Store.Q.InsertAugmentedImage(ctx, db.InsertAugmentedImageParams{
		Path:       img.Path,
		Status:     domain.StatusDone,
		FormatUsed: sql.NullString{String: img.FormatUsed, Valid: true},
		ParentID:   sql.NullInt64{Int64: img.ParentID, Valid: true},
		UpdatedAt:  sql.NullString{String: time.Now().UTC().Format(TimeFormatISO8601), Valid: true},
	})
}

// StreamAugment / StreamTrain expose the single active job's progress as SSE.
func (a *App) StreamAugment(c *gin.Context) { a.streamJob(c, "augmentation") }
func (a *App) StreamTrain(c *gin.Context)   { a.streamJob(c, "training") }

func (a *App) streamJob(c *gin.Context, name string) {
	_, ch, ok := a.Jobs.Subscribe()
	if !ok {
		problem.Write(c, http.StatusNotFound, fmt.Sprintf("No %s job running", name), "")
		return
	}

	c.Writer.Header().Set("Content-Type", "text/event-stream")
	c.Writer.Header().Set("Cache-Control", "no-cache")
	c.Writer.Header().Set("X-Accel-Buffering", "no")

	c.Stream(func(w io.Writer) bool {
		select {
		case evt, open := <-ch:
			if !open {
				return false
			}
			data, _ := json.Marshal(evt)
			fmt.Fprintf(w, "data: %s\n\n", data)
			return evt.Status != string(jobs.StatusCompleted) && evt.Status != string(jobs.StatusFailed)
		case <-time.After(SSEHeartbeatInterval):
			fmt.Fprint(w, "data: {\"heartbeat\": true}\n\n")
			return true
		case <-c.Request.Context().Done():
			return false
		}
	})
}

// emitTerminal sends the completed/failed event that closes an SSE stream.
func emitTerminal(emit jobs.EmitFunc, doneMsg string, err error) {
	if err != nil {
		emit(jobs.StatusFailed, err.Error(), map[string]any{"error": err.Error()})
		return
	}
	emit(jobs.StatusCompleted, doneMsg, map[string]any{"finished": true})
}

func progressData(p compute.Progress) map[string]any {
	return map[string]any{"stage": p.Stage, "progress": p.Progress, "details": p.Details}
}

func orStr(v, fallback string) string {
	if v == "" {
		return fallback
	}
	return v
}

func orInt(v, fallback int32) int32 {
	if v <= 0 {
		return fallback
	}
	return v
}
