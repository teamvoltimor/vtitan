package handlers

import (
	"context"
	"encoding/json"
	"net/http"
	"net/http/httptest"
	"path/filepath"
	"strings"
	"testing"
	"time"

	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/db"
	annotationdomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation"
	annotsqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/annotation/sqlite"
	computedomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute"
	computesqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/compute/sqlite"
	gallerysqlite "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/gallery/sqlite"
	gallerydomain "github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/gallery"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/job"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/internal/config"
	"github.com/teamvoldemor/voldemorbot/auto-annotator/api/domain/store"
)

// fakeClients is a stand-in compute.Clients so handlers can be tested without
// the Python ML workers.
type fakeClients struct {
	segErr    error
	augBlock  chan struct{}
	segResult computedomain.SegmentResult
}

func (f *fakeClients) Segment(_ context.Context, _ computedomain.SegmentInput) (computedomain.SegmentResult, error) {
	return f.segResult, f.segErr
}

func (f *fakeClients) RunAugmentation(
	_ context.Context,
	_ computedomain.AugmentInput,
	onProgress func(computedomain.Progress),
) error {
	if f.augBlock != nil {
		<-f.augBlock
	}
	onProgress(computedomain.Progress{Status: "running", Message: "augmenting", Progress: 1})
	return nil
}

func (f *fakeClients) RunTraining(_ context.Context, _ computedomain.TrainInput, _ func(computedomain.Progress)) error {
	return nil
}

func (f *fakeClients) Close() error { return nil }

// newTestApp opens a test DB and wires all domain services, returning the App
// and the raw query set for test data seeding.
func newTestApp(t *testing.T, fc computedomain.Clients) (*App, *db.Queries) {
	t.Helper()
	dbPath := filepath.Join(t.TempDir(), "test.db")
	st, err := store.Open(dbPath)
	if err != nil {
		t.Fatalf("open store: %v", err)
	}
	t.Cleanup(func() { _ = st.Close() })

	cfg := config.Config{DataDir: t.TempDir(), APIPublicURL: "http://localhost:8000"}
	q := st.Q

	annSvc := annotationdomain.NewService(annotsqlite.New(q), cfg)
	galSvc := gallerydomain.NewService(gallerysqlite.New(q), cfg)
	jm := job.New()
	compSvc := computedomain.NewService(fc, jm, computesqlite.New(q), cfg)

	return New(cfg, annSvc, galSvc, compSvc, st.DB), q
}

func doJSON(t *testing.T, router http.Handler, method, path, body string) *httptest.ResponseRecorder {
	t.Helper()
	req := httptest.NewRequestWithContext(context.Background(), method, path, strings.NewReader(body))
	req.Header.Set("Content-Type", "application/json")
	rec := httptest.NewRecorder()
	router.ServeHTTP(rec, req)
	return rec
}

func TestSegment(t *testing.T) {
	fc := &fakeClients{segResult: computedomain.SegmentResult{
		State:  string(computedomain.StateReady),
		Shapes: []computedomain.Shape{{ID: "mask-1-0", ClassName: "red_prism", Points: []computedomain.Point{{X: 0.1, Y: 0.2}}}},
	}}
	app, q := newTestApp(t, fc)
	if err := q.InsertImageOrIgnore(context.Background(), "/tmp/x.png"); err != nil {
		t.Fatalf("seed image: %v", err)
	}
	router := app.Router()

	t.Run("ready", func(t *testing.T) {
		rec := doJSON(t, router, http.MethodPost, "/api/v1/segment",
			`{"imageId":1,"points":[{"x":"0.1","y":"0.1","pointType":"positive","className":"red_prism"}]}`)
		if rec.Code != http.StatusOK {
			t.Fatalf("want 200, got %d: %s", rec.Code, rec.Body.String())
		}
		var resp struct {
			State  string `json:"state"`
			Shapes []struct {
				ClassName string `json:"className"`
			} `json:"shapes"`
		}
		if err := json.Unmarshal(rec.Body.Bytes(), &resp); err != nil {
			t.Fatalf("decode: %v", err)
		}
		if resp.State != string(computedomain.StateReady) || len(resp.Shapes) != 1 ||
			resp.Shapes[0].ClassName != "red_prism" {
			t.Fatalf("unexpected response: %s", rec.Body.String())
		}
	})

	t.Run("empty points -> 400", func(t *testing.T) {
		rec := doJSON(t, router, http.MethodPost, "/api/v1/segment", `{"imageId":1,"points":[]}`)
		if rec.Code != http.StatusBadRequest {
			t.Fatalf("want 400, got %d: %s", rec.Code, rec.Body.String())
		}
	})

	t.Run("unknown class -> 400", func(t *testing.T) {
		rec := doJSON(t, router, http.MethodPost, "/api/v1/segment",
			`{"imageId":1,"points":[{"x":"0.1","y":"0.1","pointType":"positive","className":"ghost"}]}`)
		if rec.Code != http.StatusBadRequest {
			t.Fatalf("want 400, got %d: %s", rec.Code, rec.Body.String())
		}
	})

	t.Run("missing image -> 404", func(t *testing.T) {
		rec := doJSON(t, router, http.MethodPost, "/api/v1/segment",
			`{"imageId":999,"points":[{"x":"0.1","y":"0.1","pointType":"positive","className":"red_prism"}]}`)
		if rec.Code != http.StatusNotFound {
			t.Fatalf("want 404, got %d: %s", rec.Code, rec.Body.String())
		}
	})
}

func TestAugmentJobLifecycle(t *testing.T) {
	fc := &fakeClients{augBlock: make(chan struct{})}
	app, q := newTestApp(t, fc)
	if err := q.InsertImageOrIgnore(context.Background(), "/tmp/x.png"); err != nil {
		t.Fatalf("seed image: %v", err)
	}
	router := app.Router()

	rec := doJSON(t, router, http.MethodPost, "/api/v1/augment/start", `{"imageIds":[1],"numAugmentations":2}`)
	if rec.Code != http.StatusAccepted {
		t.Fatalf("start: want 202, got %d: %s", rec.Code, rec.Body.String())
	}

	// Second start while the first is blocked -> 409.
	busy := doJSON(t, router, http.MethodPost, "/api/v1/augment/start", `{"imageIds":[1]}`)
	if busy.Code != http.StatusConflict {
		t.Fatalf("busy: want 409, got %d: %s", busy.Code, busy.Body.String())
	}

	// Status reports running.
	st := doJSON(t, router, http.MethodGet, "/api/v1/augment/status", "")
	if !strings.Contains(st.Body.String(), `"running":true`) {
		t.Fatalf("status: expected running true, got %s", st.Body.String())
	}

	// Release the worker and wait for the slot to clear.
	close(fc.augBlock)
	deadline := time.Now().Add(2 * time.Second)
	for app.compute.svc.JobRunning() && time.Now().Before(deadline) {
		time.Sleep(5 * time.Millisecond)
	}
	if app.compute.svc.JobRunning() {
		t.Fatal("job did not finish after worker released")
	}

	noImages := doJSON(t, router, http.MethodPost, "/api/v1/augment/start", `{"imageIds":[]}`)
	if noImages.Code != http.StatusBadRequest {
		t.Fatalf("no images: want 400, got %d", noImages.Code)
	}
}
