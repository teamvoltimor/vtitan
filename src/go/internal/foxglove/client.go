package foxglove

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"fmt"
	"net/http"
	"sync"
	"time"

	"github.com/coder/websocket"
)

// sendBufferSize bounds how many outbound frames a client's writer may lag
// behind by before frames start being dropped. Sized generously for a
// telemetry stream at typical ROBOT control-loop rates (20-30 Hz across a
// dozen-odd channels) -- a client that falls this far behind is presumed
// gone, not merely slow, and dropping keeps every OTHER client's stream
// live rather than making them all wait on it.
const sendBufferSize = 256

// encodingProtobuf is the Foxglove wire-encoding name this bridge advertises
// and stamps on every message (the Foxglove WebSocket protocol's
// "protobuf" encoding).
const encodingProtobuf = "protobuf"

// client is one connected Foxglove Studio session: its own subscription
// state (which channel IDs it wants, keyed by the SUBSCRIPTION id it
// assigned, per the protocol) and a bounded outbound frame queue drained
// by a dedicated writer goroutine, so Publish's callers never block on a
// slow network write.
type client struct {
	conn *websocket.Conn
	send chan wireFrame

	mu        sync.Mutex
	byChannel map[uint32]map[uint32]struct{} // channelID -> subscriptionIDs
}

// wireFrame is one outbound frame, tagged with the WebSocket message type
// it must be sent as -- the protocol's JSON control messages go out as
// text frames, Message Data as binary.
type wireFrame struct {
	kind websocket.MessageType
	data []byte
}

func newClient(conn *websocket.Conn) *client {
	return &client{
		conn:      conn,
		send:      make(chan wireFrame, sendBufferSize),
		byChannel: make(map[uint32]map[uint32]struct{}),
	}
}

// enqueue queues frame for this client's writer, dropping it (rather than
// blocking the caller) if the client has fallen sendBufferSize frames
// behind.
func (c *client) enqueue(frame wireFrame) {
	select {
	case c.send <- frame:
	default:
	}
}

// publishIfSubscribed enqueues frame -- rewritten with each of this
// client's own subscription IDs for channelID -- if this client currently
// subscribes to channelID. A no-op otherwise, which is the common case for
// most (channel, client) pairs at any moment.
func (c *client) publishIfSubscribed(channelID uint32, frame messageDataTemplate) {
	c.mu.Lock()
	subs := c.byChannel[channelID]
	if len(subs) == 0 {
		c.mu.Unlock()
		return
	}
	ids := make([]uint32, 0, len(subs))
	for id := range subs {
		ids = append(ids, id)
	}
	c.mu.Unlock()

	for _, subID := range ids {
		c.enqueue(wireFrame{kind: websocket.MessageBinary, data: frame.withSubscriptionID(subID)})
	}
}

func (c *client) subscribe(subs []clientSubscription) {
	c.mu.Lock()
	defer c.mu.Unlock()
	for _, s := range subs {
		if c.byChannel[s.ChannelID] == nil {
			c.byChannel[s.ChannelID] = make(map[uint32]struct{})
		}
		c.byChannel[s.ChannelID][s.ID] = struct{}{}
	}
}

func (c *client) unsubscribe(subscriptionIDs []uint32) {
	c.mu.Lock()
	defer c.mu.Unlock()
	want := make(map[uint32]struct{}, len(subscriptionIDs))
	for _, id := range subscriptionIDs {
		want[id] = struct{}{}
	}
	for channelID, subs := range c.byChannel {
		for id := range want {
			delete(subs, id)
		}
		if len(subs) == 0 {
			delete(c.byChannel, channelID)
		}
	}
}

// writeLoop drains c.send until ctx is done, writing each frame with its
// tagged message type. The sole writer for this connection -- every
// outbound frame, from any goroutine, goes through enqueue instead of
// writing directly, since coder/websocket requires writes to be
// serialized per connection.
func (c *client) writeLoop(ctx context.Context) {
	for {
		select {
		case <-ctx.Done():
			return
		case frame, ok := <-c.send:
			if !ok {
				return
			}
			if err := c.conn.Write(ctx, frame.kind, frame.data); err != nil {
				return
			}
		}
	}
}

// readLoop blocks reading client->server control messages (subscribe /
// unsubscribe) until ctx is done or the connection errors/closes.
func (c *client) readLoop(ctx context.Context) {
	for {
		_, data, err := c.conn.Read(ctx)
		if err != nil {
			return
		}
		var op clientOpMessage
		if err = json.Unmarshal(data, &op); err != nil {
			continue // a malformed control message costs this one message, not the connection
		}
		switch op.Op {
		case "subscribe":
			var msg subscribeMessage
			if json.Unmarshal(data, &msg) == nil {
				c.subscribe(msg.Subscriptions)
			}
		case "unsubscribe":
			var msg unsubscribeMessage
			if json.Unmarshal(data, &msg) == nil {
				c.unsubscribe(msg.SubscriptionIDs)
			}
		}
	}
}

// serveWS is the http.HandlerFunc backing Server.Handler.
func (s *Server) serveWS(w http.ResponseWriter, r *http.Request) {
	conn, err := websocket.Accept(w, r, &websocket.AcceptOptions{
		Subprotocols: []string{subprotocol},
	})
	if err != nil {
		s.logger.Error("foxglove: accepting websocket connection", "error", err)
		return
	}

	ctx, cancel := context.WithCancel(r.Context())
	defer cancel()

	c := newClient(conn)
	s.addClient(c)
	defer s.removeClient(c)

	info := serverInfoMessage{
		Op:                 "serverInfo",
		Name:               serverName,
		Capabilities:       []string{},
		SupportedEncodings: []string{encodingProtobuf},
	}
	infoData, err := json.Marshal(info)
	if err != nil {
		s.logger.Error("foxglove: marshaling serverInfo", "error", err)
		conn.Close(websocket.StatusInternalError, "serverInfo encode failed")
		return
	}
	c.enqueue(wireFrame{kind: websocket.MessageText, data: infoData})
	if channels := s.channelsSnapshot(); len(channels) > 0 {
		c.enqueue(advertiseFrame(channels))
	}

	var wg sync.WaitGroup
	wg.Add(1)
	go func() {
		defer wg.Done()
		c.writeLoop(ctx)
	}()

	c.readLoop(ctx) // blocks until the client disconnects or errors
	cancel()
	// c.send is deliberately never closed: Server.Publish/EnsureChannel may
	// hold a snapshot that still includes this client and call enqueue
	// concurrently with shutdown here, and closing a channel with a
	// concurrent sender panics. writeLoop already exits via ctx.Done()
	// above, and an un-closed, unreferenced channel is reclaimed by the GC
	// like any other value -- no receiver needs to observe a close.
	wg.Wait()
	conn.Close(websocket.StatusNormalClosure, "")
}

// advertiseFrame builds the "advertise" text frame for channels.
func advertiseFrame(channels []*channel) wireFrame {
	wire := make([]wireChannel, len(channels))
	for i, ch := range channels {
		wire[i] = wireChannel{
			ID:             ch.id,
			Topic:          ch.topic,
			Encoding:       encodingProtobuf,
			SchemaName:     ch.schemaName,
			Schema:         ch.schemaBase64,
			SchemaEncoding: encodingProtobuf,
		}
	}
	data, err := json.Marshal(advertiseMessage{Op: "advertise", Channels: wire})
	if err != nil {
		// wireChannel has no field that can fail to marshal (strings and a
		// uint32); a failure here would be a programming error, not a
		// runtime condition a caller can act on.
		panic(fmt.Sprintf("foxglove: marshaling advertise message: %v", err))
	}
	return wireFrame{kind: websocket.MessageText, data: data}
}

// messageDataTemplate is a Message Data frame with its subscriptionId
// field left as a hole, so one Publish call can stamp a different
// subscription ID per subscribed client without re-serializing the
// (potentially large) payload each time.
type messageDataTemplate struct {
	timestamp uint64
	payload   []byte
}

func messageDataFrame(timestamp time.Time, payload []byte) messageDataTemplate {
	return messageDataTemplate{timestamp: uint64(timestamp.UnixNano()), payload: payload} //nolint:gosec // wall-clock nanoseconds never approach uint64 overflow
}

func (t messageDataTemplate) withSubscriptionID(subscriptionID uint32) []byte {
	out := make([]byte, 1+messageDataHeaderLen+len(t.payload))
	out[0] = binaryOpcodeMessageData
	binary.LittleEndian.PutUint32(out[1:5], subscriptionID)
	binary.LittleEndian.PutUint64(out[5:13], t.timestamp)
	copy(out[13:], t.payload)
	return out
}
