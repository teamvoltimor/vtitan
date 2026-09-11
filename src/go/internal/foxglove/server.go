package foxglove

import (
	"encoding/base64"
	"fmt"
	"log/slog"
	"net/http"
	"sync"
	"time"

	"google.golang.org/protobuf/proto"

	"github.com/teamvoltimor/vtitan/src/go/internal/schema/protoschema"
)

// channel is one registered subject/schema pair, matching the protocol's
// Channel object plus the pre-computed fields Publish needs on every call.
type channel struct {
	id           uint32
	topic        string
	schemaName   string
	schemaBase64 string
}

// Server is a Foxglove WebSocket protocol server: any number of subjects
// can be registered via EnsureChannel and fed data via Publish, and any
// number of Foxglove Studio clients may connect concurrently via Handler.
//
// Registering a channel and publishing to it are independent of whether
// any client is currently connected -- a channel registered before the
// first client connects is included in that client's initial advertise;
// one registered after is advertised to every ALREADY-connected client
// too, matching the protocol's "advertise can arrive at any time" model.
type Server struct {
	logger *slog.Logger

	mu      sync.RWMutex
	byTopic map[string]*channel
	byID    map[uint32]*channel
	nextID  uint32
	clients map[*client]struct{}
}

// serverName is reported in the "serverInfo" handshake message.
const serverName = "vtitan-foxglove-bridge"

// NewServer returns a Server ready to accept connections via Handler.
func NewServer(logger *slog.Logger) *Server {
	return &Server{
		logger:  logger,
		byTopic: make(map[string]*channel),
		byID:    make(map[uint32]*channel),
		clients: make(map[*client]struct{}),
	}
}

// EnsureChannel registers subject as a channel if not already registered,
// using msg's protobuf descriptor (a zero-value instance is fine -- only
// its type is inspected) for the schema every client needs to decode
// future Publish payloads. Returns the channel's stable ID either way.
func (s *Server) EnsureChannel(subject string, msg proto.Message) (uint32, error) {
	s.mu.Lock()
	if ch, ok := s.byTopic[subject]; ok {
		s.mu.Unlock()
		return ch.id, nil
	}

	descriptorSet, err := protoschema.FileDescriptorSet(msg)
	if err != nil {
		s.mu.Unlock()
		return 0, fmt.Errorf("foxglove: building schema for %s: %w", subject, err)
	}

	id := s.nextID
	s.nextID++
	ch := &channel{
		id:           id,
		topic:        subject,
		schemaName:   string(proto.MessageName(msg)),
		schemaBase64: base64.StdEncoding.EncodeToString(descriptorSet),
	}
	s.byTopic[subject] = ch
	s.byID[id] = ch

	snapshot := s.clientSnapshotLocked()
	s.mu.Unlock()

	frame := advertiseFrame([]*channel{ch})
	for _, c := range snapshot {
		c.enqueue(frame)
	}
	return id, nil
}

// Publish forwards payload (an already wire-encoded protobuf message) as a
// Message Data frame to every client currently subscribed to channelID.
// A client that never subscribed, or that hasn't connected at all, costs
// nothing -- Publish never blocks on a slow or absent client.
func (s *Server) Publish(channelID uint32, timestamp time.Time, payload []byte) {
	frame := messageDataFrame(timestamp, payload)

	s.mu.RLock()
	snapshot := s.clientSnapshotLocked()
	s.mu.RUnlock()

	for _, c := range snapshot {
		c.publishIfSubscribed(channelID, frame)
	}
}

// Handler returns the HTTP handler that upgrades incoming requests to the
// Foxglove WebSocket protocol and serves them until the client disconnects.
func (s *Server) Handler() http.Handler {
	return http.HandlerFunc(s.serveWS)
}

// clientSnapshotLocked returns the current client set as a slice, callable
// with s.mu held (read or write) so the caller can release the lock before
// touching any client -- sending to a client's channel while holding
// Server.mu would let one slow client stall every other Server method.
func (s *Server) clientSnapshotLocked() []*client {
	out := make([]*client, 0, len(s.clients))
	for c := range s.clients {
		out = append(out, c)
	}
	return out
}

// channelsSnapshot returns every currently registered channel, for a
// freshly connected client's initial advertise.
func (s *Server) channelsSnapshot() []*channel {
	s.mu.RLock()
	defer s.mu.RUnlock()
	out := make([]*channel, 0, len(s.byID))
	for _, ch := range s.byID {
		out = append(out, ch)
	}
	return out
}

func (s *Server) addClient(c *client) {
	s.mu.Lock()
	defer s.mu.Unlock()
	s.clients[c] = struct{}{}
}

func (s *Server) removeClient(c *client) {
	s.mu.Lock()
	defer s.mu.Unlock()
	delete(s.clients, c)
}
