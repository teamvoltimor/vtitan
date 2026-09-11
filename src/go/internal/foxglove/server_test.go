package foxglove_test

import (
	"context"
	"encoding/binary"
	"encoding/json"
	"log/slog"
	"net/http/httptest"
	"testing"
	"time"

	"github.com/coder/websocket"

	"github.com/teamvoltimor/vtitan/src/go/internal/foxglove"
	"github.com/teamvoltimor/vtitan/src/go/internal/foxglove/foxglovetest"
	sensorv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/sensor/v1"
)

const testTimeout = 5 * time.Second

func newTestServer(t *testing.T) (*foxglove.Server, *httptest.Server) {
	t.Helper()
	s := foxglove.NewServer(slog.New(slog.DiscardHandler))
	httpSrv := httptest.NewServer(s.Handler())
	t.Cleanup(httpSrv.Close)
	return s, httpSrv
}

// dial connects a raw websocket client to httpSrv using the Foxglove
// subprotocol, matching what Foxglove Studio's own client negotiates.
func dial(t *testing.T, ctx context.Context, wsURL string) *websocket.Conn {
	t.Helper()
	conn, _, err := websocket.Dial(ctx, wsURL, &websocket.DialOptions{
		Subprotocols: []string{"foxglove.websocket.v1"},
	})
	if err != nil {
		t.Fatalf("websocket.Dial: %v", err)
	}
	t.Cleanup(func() { conn.Close(websocket.StatusNormalClosure, "") })
	return conn
}

// readJSON reads the next TEXT frame and decodes it as JSON into v.
func readJSON(t *testing.T, ctx context.Context, conn *websocket.Conn, v any) {
	t.Helper()
	kind, data, err := conn.Read(ctx)
	if err != nil {
		t.Fatalf("conn.Read: %v", err)
	}
	if kind != websocket.MessageText {
		t.Fatalf("conn.Read: kind = %v, want MessageText", kind)
	}
	if err = json.Unmarshal(data, v); err != nil {
		t.Fatalf("json.Unmarshal(%s): %v", data, err)
	}
}

func TestServeWS_SendsServerInfoOnConnect(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	_, httpSrv := newTestServer(t)
	conn := dial(t, ctx, wsURL(httpSrv.URL))

	var info map[string]any
	readJSON(t, ctx, conn, &info)
	if info["op"] != "serverInfo" {
		t.Errorf(`first message op = %v, want "serverInfo"`, info["op"])
	}
	if info["name"] == "" || info["name"] == nil {
		t.Error("serverInfo has no name")
	}
}

func TestServeWS_AdvertisesAlreadyRegisteredChannelsOnConnect(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	s, httpSrv := newTestServer(t)
	if _, err := s.EnsureChannel(sensorv1.ImuSubject, &sensorv1.Imu{}); err != nil {
		t.Fatalf("EnsureChannel: %v", err)
	}

	conn := dial(t, ctx, wsURL(httpSrv.URL))
	var info map[string]any
	readJSON(t, ctx, conn, &info) // serverInfo

	var advertise foxglovetest.AdvertiseResponse
	readJSON(t, ctx, conn, &advertise)
	if advertise.Op != "advertise" {
		t.Fatalf(`second message op = %v, want "advertise"`, advertise.Op)
	}
	if len(advertise.Channels) != 1 || advertise.Channels[0].Topic != sensorv1.ImuSubject {
		t.Errorf("advertise.channels = %+v, want one channel for %s", advertise.Channels, sensorv1.ImuSubject)
	}
}

func TestServeWS_AdvertisesANewChannelToAnAlreadyConnectedClient(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	s, httpSrv := newTestServer(t)
	conn := dial(t, ctx, wsURL(httpSrv.URL))
	var info map[string]any
	readJSON(t, ctx, conn, &info) // serverInfo, no channels registered yet

	if _, err := s.EnsureChannel(sensorv1.ImuSubject, &sensorv1.Imu{}); err != nil {
		t.Fatalf("EnsureChannel: %v", err)
	}

	var advertise foxglovetest.AdvertiseResponse
	readJSON(t, ctx, conn, &advertise)
	if advertise.Op != "advertise" || len(advertise.Channels) != 1 {
		t.Fatalf("advertise = %+v, want one newly-registered channel", advertise)
	}
}

// TestPublish_SubscribedClientReceivesMessageDataWithItsOwnSubscriptionID
// is the core end-to-end contract: subscribe, then Publish, and the client
// must receive a binary Message Data frame keyed by the SUBSCRIPTION id it
// chose (not the channel id), carrying the raw payload bytes unchanged.
func TestPublish_SubscribedClientReceivesMessageDataWithItsOwnSubscriptionID(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	s, httpSrv := newTestServer(t)
	channelID, err := s.EnsureChannel(sensorv1.ImuSubject, &sensorv1.Imu{})
	if err != nil {
		t.Fatalf("EnsureChannel: %v", err)
	}

	conn := dial(t, ctx, wsURL(httpSrv.URL))
	var info map[string]any
	readJSON(t, ctx, conn, &info) // serverInfo
	var advertise foxglovetest.AdvertiseResponse
	readJSON(t, ctx, conn, &advertise) // advertise

	const subscriptionID uint32 = 42
	subMsg, _ := json.Marshal(map[string]any{
		"op": "subscribe",
		"subscriptions": []map[string]any{
			{"id": subscriptionID, "channelId": channelID},
		},
	})
	if err = conn.Write(ctx, websocket.MessageText, subMsg); err != nil {
		t.Fatalf("conn.Write(subscribe): %v", err)
	}

	// Publish() has no ack, and re-deriving a fresh per-attempt context for
	// each Read would cancel the WHOLE connection the instant one attempt's
	// context expires (coder/websocket ties the connection's lifetime to
	// the context passed to Read/Write, not just that one call) -- so read
	// ONCE against the real long-lived ctx in the background, and keep
	// Publish()ing from the foreground until the subscription has taken
	// effect server-side or that read returns.
	resultCh := make(chan foxglovetest.ReadResult, 1)
	go func() {
		kind, data, err := conn.Read(ctx)
		resultCh <- foxglovetest.ReadResult{Kind: kind, Data: data, Err: err}
	}()

	payload := []byte{0xDE, 0xAD, 0xBE, 0xEF}
	stamp := time.Unix(1700000000, 123456789)
	ticker := time.NewTicker(20 * time.Millisecond)
	defer ticker.Stop()

	var frame []byte
	for frame == nil {
		select {
		case res := <-resultCh:
			if res.Err != nil {
				t.Fatalf("conn.Read: %v", res.Err)
			}
			if res.Kind != websocket.MessageBinary {
				t.Fatalf("conn.Read: kind = %v, want MessageBinary", res.Kind)
			}
			frame = res.Data
		case <-ticker.C:
			s.Publish(channelID, stamp, payload)
		case <-ctx.Done():
			t.Fatal("never received a Message Data frame after subscribing")
		}
	}

	if frame[0] != 0x01 {
		t.Errorf("frame[0] (opcode) = %v, want 0x01", frame[0])
	}
	gotSubID := binary.LittleEndian.Uint32(frame[1:5])
	if gotSubID != subscriptionID {
		t.Errorf("frame subscriptionId = %v, want %v", gotSubID, subscriptionID)
	}
	gotTimestamp := binary.LittleEndian.Uint64(frame[5:13])
	if gotTimestamp != uint64(stamp.UnixNano()) {
		t.Errorf("frame timestamp = %v, want %v", gotTimestamp, stamp.UnixNano())
	}
	gotPayload := frame[13:]
	if string(gotPayload) != string(payload) {
		t.Errorf("frame payload = %v, want %v", gotPayload, payload)
	}
}

func TestPublish_UnsubscribedClientReceivesNothing(t *testing.T) {
	t.Parallel()
	ctx, cancel := context.WithTimeout(context.Background(), testTimeout)
	defer cancel()

	s, httpSrv := newTestServer(t)
	channelID, err := s.EnsureChannel(sensorv1.ImuSubject, &sensorv1.Imu{})
	if err != nil {
		t.Fatalf("EnsureChannel: %v", err)
	}

	conn := dial(t, ctx, wsURL(httpSrv.URL))
	var info map[string]any
	readJSON(t, ctx, conn, &info)
	var advertise foxglovetest.AdvertiseResponse
	readJSON(t, ctx, conn, &advertise)

	// No subscribe message sent.
	s.Publish(channelID, time.Now(), []byte{0x01})

	readCtx, readCancel := context.WithTimeout(ctx, 200*time.Millisecond)
	defer readCancel()
	if _, _, readErr := conn.Read(readCtx); readErr == nil {
		t.Error("received a frame despite never subscribing")
	}
}

// wsURL rewrites an httptest.Server's http:// URL to ws://, matching what
// a real websocket client dials.
func wsURL(httpURL string) string {
	return "ws" + httpURL[len("http"):]
}
