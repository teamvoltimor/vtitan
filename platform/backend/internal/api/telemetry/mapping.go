package telemetry

import (
	"encoding/json"
	"log/slog"
	"net/http"

	"github.com/gin-gonic/gin"
	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/platform/backend/internal/problem"
)

// contentTypeJSON is the content type for proto-marshaled JSON responses.
const contentTypeJSON = "application/json; charset=utf-8"

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = true forces zero-valued *implicit*-presence fields
// (e.g. angular_velocity.x/y/z when the IMU reads exactly 0) onto the wire
// instead of omitting them -- the frontend's schema requires these numeric
// fields always be present. This does NOT affect proto3 `optional` fields
// (range_min, speed, etc.): those are backed by a synthetic oneof, which
// EmitUnpopulated explicitly excludes, so a genuinely-unset optional still
// doesn't get force-emitted -- the "not measured" vs "measured as zero"
// distinction those fields rely on is preserved.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: true, UseProtoNames: true}

// writeProto marshals a single proto message to JSON and writes it to the response.
func writeProto(c *gin.Context, code int, msg proto.Message) {
	b, err := marshaler.Marshal(msg)
	if err != nil {
		slog.Error("proto marshal", "error", err)
		problem.InternalError(c)
		return
	}
	c.Data(code, contentTypeJSON, b)
}

// writeProtoSlice marshals a slice of proto messages to a JSON array.
func writeProtoSlice[T proto.Message](c *gin.Context, msgs []T) {
	parts := make([]json.RawMessage, len(msgs))
	for i, msg := range msgs {
		b, err := marshaler.Marshal(msg)
		if err != nil {
			slog.Error("proto marshal slice item", "error", err, "index", i)
			problem.InternalError(c)
			return
		}
		parts[i] = b
	}
	out, err := json.Marshal(parts)
	if err != nil {
		slog.Error("json marshal slice", "error", err)
		problem.InternalError(c)
		return
	}
	c.Data(http.StatusOK, contentTypeJSON, out)
}
