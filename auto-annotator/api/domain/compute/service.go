package compute

import (
	"context"
	"fmt"
	"log/slog"
	"path/filepath"

	"github.com/BurntSushi/toml"

	"github.com/teamvoltimor/vtitan/auto-annotator/api/domain/job"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/config"
	"github.com/teamvoltimor/vtitan/auto-annotator/api/internal/domain"
)

const (
	defaultTrainEpochs = int32(domain.DefaultTrainEpochs)
	defaultTrainBatch  = int32(domain.DefaultTrainBatch)
	defaultTrainImgsz  = int32(domain.DefaultTrainImgsz)
)

type modelsConfig struct {
	DefaultModel string `toml:"default_model"`
	Models       []struct {
		ID           string `toml:"id"`
		Label        string `toml:"label"`
		Type         string `toml:"type"`
		SupportsText bool   `toml:"supports_text"`
	} `toml:"models"`
}

type computeService struct {
	clients Clients
	jobs    *job.Manager
	store   Store
	cfg     config.Config
}

// NewService returns a compute Service wired to the given clients, job manager, store and config.
func NewService(clients Clients, jm *job.Manager, store Store, cfg config.Config) Service {
	return &computeService{clients: clients, jobs: jm, store: store, cfg: cfg}
}

func (s *computeService) ListModels() ([]Model, error) {
	var cfg modelsConfig
	if _, err := toml.DecodeFile(s.cfg.ModelsConfig, &cfg); err != nil {
		// A missing/malformed models.toml is an operator misconfiguration,
		// not "zero models configured" -- the caller still gets an empty
		// list (unchanged behavior), but this is no longer silent.
		slog.Warn("failed to load models config, returning empty model list",
			"path", s.cfg.ModelsConfig, "error", err)
		return []Model{}, nil
	}
	items := make([]Model, 0, len(cfg.Models))
	for _, m := range cfg.Models {
		label := m.Label
		if label == "" {
			label = m.ID
		}
		items = append(items, Model{
			ID:           m.ID,
			Label:        label,
			ModelType:    m.Type,
			Available:    true,
			Active:       m.ID == cfg.DefaultModel,
			SupportsText: m.SupportsText,
		})
	}
	return items, nil
}

func (s *computeService) Segment(ctx context.Context, req SegmentReq) (SegmentResult, error) {
	img, err := s.store.GetImage(ctx, req.ImageID)
	if err != nil {
		return SegmentResult{}, fmt.Errorf("image %d: %w", req.ImageID, domain.ErrNotFound)
	}

	classNames, err := s.store.ListClassNames(ctx)
	if err != nil {
		return SegmentResult{}, err
	}
	if len(req.Points) == 0 {
		return SegmentResult{}, fmt.Errorf("%s: %w", domain.ErrAtLeastOnePoint, domain.ErrInvalidInput)
	}
	known := make(map[string]bool, len(classNames))
	for _, name := range classNames {
		known[name] = true
	}
	for _, p := range req.Points {
		if !known[p.ClassName] {
			// domain.ErrFmtUnknownClass is capitalized intentionally: problem.FromDomain
			// echoes err.Error() verbatim as the HTTP Problem Detail "detail" field, so
			// this is user-facing API response text, not just an internal wrapped error.
			return SegmentResult{}, fmt.Errorf(domain.ErrFmtUnknownClass+": %w", //nolint:staticcheck
				p.ClassName, domain.ErrInvalidInput)
		}
	}

	return s.clients.Segment(ctx, SegmentInput{
		ImageID:    img.ID,
		ImagePath:  img.Path,
		Points:     req.Points,
		ClassNames: classNames,
	})
}

func (s *computeService) StartAugment(ctx context.Context, req AugmentJobReq) error {
	if len(req.ImageIDs) == 0 {
		return fmt.Errorf("%s: %w", domain.ErrNoImagesSpecified, domain.ErrInvalidInput)
	}
	if s.jobs.Running() {
		return fmt.Errorf("%s: %w", domain.ErrAugmentationRunning, domain.ErrConflict)
	}

	num := req.NumAugmentations
	if num <= 0 {
		num = domain.DefaultNumAugmentations
	}

	sources := make([]AugmentSource, 0, len(req.ImageIDs))
	for _, id := range req.ImageIDs {
		img, err := s.store.GetImage(ctx, id)
		if err != nil {
			continue
		}
		format := img.FormatUsed
		if format == "" {
			format = "det"
		}
		sources = append(sources, AugmentSource{ImageID: img.ID, Path: img.Path, FormatUsed: format})
	}

	// jobCtx is a fresh context owned by the job manager, not the incoming request's
	// ctx: the job must keep running after this HTTP handler returns.
	if _, err := s.jobs.Start(func(jobCtx context.Context, emit job.EmitFunc) { //nolint:contextcheck
		s.runAugment(jobCtx, emit, AugmentInput{Sources: sources, NumAugmentations: num})
	}); err != nil {
		return fmt.Errorf("%s: %w", domain.ErrAugmentationRunning, domain.ErrConflict)
	}
	return nil
}

func (s *computeService) StartTrain(ctx context.Context, req TrainJobReq) error {
	if s.jobs.Running() {
		return fmt.Errorf("%s: %w", domain.ErrTrainingRunning, domain.ErrConflict)
	}
	in := TrainInput{
		ModelName:    orStr(req.ModelName, domain.DefaultTrainModel),
		Epochs:       orInt(req.Epochs, defaultTrainEpochs),
		Batch:        orInt(req.Batch, defaultTrainBatch),
		Imgsz:        orInt(req.Imgsz, defaultTrainImgsz),
		DataYamlPath: filepath.Join(s.cfg.DataDir, domain.DataYAMLName),
	}
	// jobCtx is a fresh context owned by the job manager, not the incoming request's
	// ctx: the job must keep running after this HTTP handler returns.
	if _, err := s.jobs.Start(func(jobCtx context.Context, emit job.EmitFunc) { //nolint:contextcheck
		s.runTrain(jobCtx, emit, in)
	}); err != nil {
		return fmt.Errorf("%s: %w", domain.ErrTrainingRunning, domain.ErrConflict)
	}
	return nil
}

func (s *computeService) JobRunning() bool { return s.jobs.Running() }

func (s *computeService) Subscribe() (string, <-chan job.Event, bool) {
	return s.jobs.Subscribe()
}

func (s *computeService) runAugment(ctx context.Context, emit job.EmitFunc, in AugmentInput) {
	err := s.clients.RunAugmentation(ctx, in, func(p Progress) {
		if p.Augmented != nil {
			_ = s.store.InsertAugmented(ctx, *p.Augmented)
		}
		if p.Error == "" {
			emit(job.StatusRunning, p.Message, progressData(p))
		}
	})
	emitTerminal(emit, domain.MsgAugmentationCompleted, err)
}

func (s *computeService) runTrain(ctx context.Context, emit job.EmitFunc, in TrainInput) {
	err := s.clients.RunTraining(ctx, in, func(p Progress) {
		if p.Error == "" {
			emit(job.StatusRunning, p.Message, progressData(p))
		}
	})
	emitTerminal(emit, domain.MsgTrainingCompleted, err)
}

func emitTerminal(emit job.EmitFunc, doneMsg string, err error) {
	if err != nil {
		emit(job.StatusFailed, err.Error(), map[string]any{domain.MapKeyError: err.Error()})
		return
	}
	emit(job.StatusCompleted, doneMsg, map[string]any{domain.MapKeyFinished: true})
}

func progressData(p Progress) map[string]any {
	return map[string]any{
		domain.MapKeyStage:    p.Stage,
		domain.MapKeyProgress: p.Progress,
		domain.MapKeyDetails:  p.Details,
	}
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
