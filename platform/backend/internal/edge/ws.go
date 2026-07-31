package edge

import (
	"context"
	"log/slog"
	"time"

	"github.com/coder/websocket"
	"github.com/gin-gonic/gin"

	"github.com/teamvoltimor/vtitan/platform/backend/domain/telemetry"
	telemetryv1 "github.com/teamvoltimor/vtitan/platform/backend/gen/telemetry/v1"
)

const wsWriteTimeout = 5 * time.Second

type wsManager struct {
	telSvc telemetry.TelemetryService
}

func newWSManager(telSvc telemetry.TelemetryService) *wsManager {
	return &wsManager{telSvc: telSvc}
}

// handle upgrades an HTTP connection to WebSocket and streams snapshots until
// the client disconnects or the context is canceled.
func (m *wsManager) handle(c *gin.Context) {
	conn, err := websocket.Accept(c.Writer, c.Request, &websocket.AcceptOptions{
		// CORS is enforced by the corsMiddleware on the gin engine.
		InsecureSkipVerify: true,
	})
	if err != nil {
		slog.Warn("ws accept failed", "error", err)
		return
	}
	defer func() { _ = conn.CloseNow() }()

	// CloseRead drains the read side and returns a context that is canceled
	// when the client sends a close frame or disconnects.
	ctx := conn.CloseRead(c.Request.Context())

	ch, unsub := m.telSvc.Subscribe()
	defer unsub()

	for {
		select {
		case <-ctx.Done():
			conn.Close(websocket.StatusNormalClosure, "")
			return

		case snap, ok := <-ch:
			if !ok {
				return
			}
			if err := m.send(ctx, conn, snap); err != nil {
				slog.Debug("ws write error", "error", err)
				return
			}
		}
	}
}

func (m *wsManager) send(ctx context.Context, conn *websocket.Conn, snap *telemetryv1.RobotSnapshot) error {
	b, err := marshaler.Marshal(snap)
	if err != nil {
		slog.Error("ws marshal snapshot", "error", err)
		return nil // marshal error is not a connection error; skip the frame
	}
	writeCtx, cancel := context.WithTimeout(ctx, wsWriteTimeout)
	defer cancel()
	return conn.Write(writeCtx, websocket.MessageText, b)
}

// now is a package-level var so handlers_test.go can stub it.
var now = time.Now
