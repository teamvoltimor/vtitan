package telemetry

import (
	"encoding/json"
	"net/http"

	"github.com/gin-gonic/gin"
	"go.uber.org/zap"
	"google.golang.org/protobuf/encoding/protojson"
	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/platform/backend/internal/problem"
)

// contentTypeJSON is the content type for proto-marshaled JSON responses.
const contentTypeJSON = "application/json; charset=utf-8"

// marshaler serializes proto messages to snake_case JSON for the frontend.
// EmitUnpopulated = false omits zero-value optional fields.
var marshaler = protojson.MarshalOptions{EmitUnpopulated: false, UseProtoNames: true}

// writeProto marshals a single proto message to JSON and writes it to the response.
func writeProto(c *gin.Context, code int, msg proto.Message, log *zap.Logger) {
	b, err := marshaler.Marshal(msg)
	if err != nil {
		log.Error("proto marshal", zap.Error(err))
		problem.InternalError(c)
		return
	}
	c.Data(code, contentTypeJSON, b)
}

// writeProtoSlice marshals a slice of proto messages to a JSON array.
func writeProtoSlice[T proto.Message](c *gin.Context, msgs []T, log *zap.Logger) {
	parts := make([]json.RawMessage, len(msgs))
	for i, msg := range msgs {
		b, err := marshaler.Marshal(msg)
		if err != nil {
			log.Error("proto marshal slice item", zap.Error(err), zap.Int("index", i))
			problem.InternalError(c)
			return
		}
		parts[i] = b
	}
	out, err := json.Marshal(parts)
	if err != nil {
		log.Error("json marshal slice", zap.Error(err))
		problem.InternalError(c)
		return
	}
	c.Data(http.StatusOK, contentTypeJSON, out)
}
