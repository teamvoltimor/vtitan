package handlers

import (
	"encoding/json"
	"fmt"
	"io"
	"net/http"
	"time"

	"github.com/gin-gonic/gin"

	computedomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/job"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/http/problem"
)

// RegisterRoutes wires the model listing, segmentation, and augment/train job
// routes onto rg.
func (h *ComputeHandler) RegisterRoutes(rg *gin.RouterGroup) {
	rg.GET(RouteListModels, h.ListModels)
	rg.POST(RouteSegment, h.Segment)

	rg.POST(RouteStartAugment, h.StartAugment)
	rg.GET(RouteStatusAugment, h.JobStatus)
	rg.GET(RouteStreamAugment, h.StreamAugment)

	rg.POST(RouteStartTrain, h.StartTrain)
	rg.GET(RouteStatusTrain, h.JobStatus)
	rg.GET(RouteStreamTrain, h.StreamTrain)
}

// JobStatus reports whether any job is running. GET /augment/status, /train/status
func (h *ComputeHandler) JobStatus(c *gin.Context) {
	c.JSON(http.StatusOK, JobStatusResponse{Running: h.svc.JobRunning()})
}

// StartAugment launches an augmentation job. POST /augment/start
func (h *ComputeHandler) StartAugment(c *gin.Context) {
	var req AugmentRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	if err := h.svc.StartAugment(c.Request.Context(), computedomain.AugmentJobReq{
		ImageIDs:         req.ImageIds,
		NumAugmentations: req.NumAugmentations,
	}); err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusAccepted, JobStatusResponse{Running: true, Message: MsgAugmentationStarted})
}

// StartTrain launches a YOLO training job. POST /train/start
func (h *ComputeHandler) StartTrain(c *gin.Context) {
	var req TrainRequest
	if err := c.ShouldBindJSON(&req); err != nil {
		problem.ValidationError(c, err)
		return
	}
	if err := h.svc.StartTrain(c.Request.Context(), computedomain.TrainJobReq{
		ModelName: req.ModelName,
		Epochs:    req.Epochs,
		Batch:     req.Batch,
		Imgsz:     req.Imgsz,
	}); err != nil {
		problem.FromDomain(c, err)
		return
	}
	c.JSON(http.StatusAccepted, JobStatusResponse{Running: true, Message: MsgTrainingStarted})
}

// StreamAugment exposes the active augmentation job's progress as SSE. GET /augment/stream
func (h *ComputeHandler) StreamAugment(c *gin.Context) {
	h.streamJob(c, "augmentation")
}

// StreamTrain exposes the active training job's progress as SSE. GET /train/stream
func (h *ComputeHandler) StreamTrain(c *gin.Context) {
	h.streamJob(c, "training")
}

func (h *ComputeHandler) streamJob(c *gin.Context, name string) {
	_, ch, ok := h.svc.Subscribe()
	if !ok {
		problem.Write(c, http.StatusNotFound, fmt.Sprintf(ErrFmtNoJobRunning, name), "")
		return
	}

	c.Writer.Header().Set(HeaderContentType, HeaderValueEventStream)
	c.Writer.Header().Set(HeaderCacheControl, HeaderValueNoCache)
	c.Writer.Header().Set(HeaderXAccelBuffering, HeaderValueBufferingOff)

	c.Stream(func(w io.Writer) bool {
		select {
		case evt, open := <-ch:
			if !open {
				return false
			}
			data, _ := json.Marshal(evt)
			fmt.Fprintf(w, SSEDataFormat, data)
			return evt.Status != string(job.StatusCompleted) && evt.Status != string(job.StatusFailed)
		case <-time.After(SSEHeartbeatInterval):
			fmt.Fprint(w, SSEHeartbeatFormat)
			return true
		case <-c.Request.Context().Done():
			return false
		}
	})
}
