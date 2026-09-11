package foxglove

// subprotocol is the WebSocket subprotocol Foxglove Studio negotiates for
// this protocol version, matching the spec's own name.
const subprotocol = "foxglove.websocket.v1"

// binaryOpcodeMessageData is the first byte of every server->client BINARY
// frame this package sends -- the protocol also defines opcodes for
// service-call responses and time updates, neither of which this
// one-directional telemetry bridge uses.
const binaryOpcodeMessageData byte = 0x01

// messageDataHeaderLen is subscriptionId (4 bytes) + timestamp (8 bytes)
// preceding the payload in a Message Data frame.
const messageDataHeaderLen = 4 + 8

// serverInfoMessage is the first message a server sends after a client
// connects, matching the "serverInfo" server->client JSON message.
type serverInfoMessage struct {
	Op           string   `json:"op"`
	Name         string   `json:"name"`
	Capabilities []string `json:"capabilities"`
	// SupportedEncodings limits which message encodings this server ever
	// advertises a channel with -- protobuf only, since every NATS
	// subject this bridge forwards is already protobuf-encoded on the
	// wire (see internal/transport/nats.Publisher).
	SupportedEncodings []string `json:"supportedEncodings"`
}

// wireChannel is one entry of the "advertise" message's channels array,
// matching the protocol's Channel object.
type wireChannel struct {
	ID             uint32 `json:"id"`
	Topic          string `json:"topic"`
	Encoding       string `json:"encoding"`
	SchemaName     string `json:"schemaName"`
	Schema         string `json:"schema"`
	SchemaEncoding string `json:"schemaEncoding"`
}

// advertiseMessage announces one or more channels to a client, matching
// the "advertise" server->client JSON message.
type advertiseMessage struct {
	Op       string        `json:"op"`
	Channels []wireChannel `json:"channels"`
}

// unadvertiseMessage retracts previously advertised channels, matching the
// "unadvertise" server->client JSON message. Not currently sent (channels
// live for the process lifetime), but decoded defensively is unnecessary
// since this package never needs to PARSE it -- kept only as a documented
// non-use, not implemented.

// clientOpMessage is the minimal shape every client->server JSON message
// shares, used to dispatch on Op before decoding the rest.
type clientOpMessage struct {
	Op string `json:"op"`
}

// clientSubscription is one entry of a "subscribe" message's subscriptions
// array: the client's own chosen ID for this subscription, paired with the
// channel it wants.
type clientSubscription struct {
	ID        uint32 `json:"id"`
	ChannelID uint32 `json:"channelId"`
}

// subscribeMessage matches the "subscribe" client->server JSON message.
type subscribeMessage struct {
	Op            string               `json:"op"`
	Subscriptions []clientSubscription `json:"subscriptions"`
}

// unsubscribeMessage matches the "unsubscribe" client->server JSON
// message: the SUBSCRIPTION ids (the client's own, from a prior
// subscribeMessage), not channel ids.
type unsubscribeMessage struct {
	Op              string   `json:"op"`
	SubscriptionIDs []uint32 `json:"subscriptionIds"`
}
