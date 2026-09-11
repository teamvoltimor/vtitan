//go:build hw

package nats_test

import (
	"context"
	"os"
	"os/exec"
	"testing"
	"time"

	natsgo "github.com/nats-io/nats.go"

	"github.com/teamvoltimor/vtitan/platform/robot-go/internal/transport/nats"
)

// TestHW_NATS_ConnectAndReconnectOverBrokerRestart is the field validation of
// the NATS deployment hardening (go-migration-plan #2/#3): it proves a real
// Go client using internal/transport/nats.Connect (a) reaches the live
// nats-server started by vtitan-nats.service, and (b) survives a full broker
// restart -- the exact failure mode of a USB-gadget link flap or a broker
// crash -- by reconnecting and resuming pub/sub.
//
// PASS -> initial connect + pub/sub works, `systemctl restart vtitan-nats`
//         drops the link, and the client reconnects and pub/sub works again.
// FAIL -> cannot reach the broker, or the client does NOT recover after the
//         restart (the reconnect hardening is broken against real hardware).
//
// Requires: vtitan-nats.service running on the host, and the caller able to
// `systemctl restart` it (root or passwordless sudo). Override the broker
// URL with VTITAN_NATS_URL (else nats.DefaultDevURL = nats://127.0.0.1:4222).
func TestHW_NATS_ConnectAndReconnectOverBrokerRestart(t *testing.T) {
	url := nats.DefaultURL()
	cfg := nats.DefaultConfig(url, "hw-reconnect-test")
	cfg.Logger = nil // keep output quiet; we assert programmatically

	ctx := context.Background()
	conn, err := nats.Connect(ctx, cfg)
	if err != nil {
		t.Fatalf("HW FAIL: initial connect to %s: %v", url, err)
	}
	defer conn.Close()

	const subject = "vtitan.hw.test"
	if !roundTrip(t, conn, subject) {
		t.Fatalf("HW FAIL: initial pub/sub round-trip on %s failed", subject)
	}
	t.Logf("HW PASS: initial connect + pub/sub on %s", url)

	// Restart the broker out from under the client -- the real flap case.
	// Skip when NATS_HW_SKIP_RESTART=1 (e.g. running on the Pi Zero, where the
	// broker lives on the Pi 5 over the gadget link and cannot be restarted
	// from here) -- in that case we only validate cross-board connect+pub/sub.
	if os.Getenv("NATS_HW_SKIP_RESTART") != "1" {
		restart := exec.Command("sudo", "systemctl", "restart", "vtitan-nats.service")
		if out, err := restart.CombinedOutput(); err != nil {
			t.Fatalf("HW FAIL: restarting vtitan-nats.service: %v\n%s", err, out)
		}
		t.Log("restarted vtitan-nats.service; waiting for client to detect + reconnect...")
	} else {
		t.Log("NATS_HW_SKIP_RESTART=1: skipping broker restart (cross-board connect+pub/sub only)")
		return
	}

	// Give nats.go time to see the drop and reconnect (PingInterval=10s,
	// ReconnectWait=2s). Poll for IsConnected rather than sleeping blind.
	reconnected := false
	deadline := time.Now().Add(45 * time.Second)
	for time.Now().Before(deadline) {
		if conn.IsConnected() {
			reconnected = true
			break
		}
		time.Sleep(500 * time.Millisecond)
	}
	if !reconnected {
		t.Fatalf("HW FAIL: client did not reconnect within 45s after broker restart (IsConnected=%v)", conn.IsConnected())
	}
	t.Log("HW PASS: client reconnected after broker restart")

	// Confirm pub/sub actually works again on the reconnected link.
	if !roundTrip(t, conn, subject) {
		t.Fatalf("HW FAIL: pub/sub round-trip after reconnect on %s failed", subject)
	}
	t.Log("HW PASS: pub/sub resumed after reconnect -- NATS deployment hardening verified")
}

func roundTrip(t *testing.T, conn *natsgo.Conn, subject string) bool {
	t.Helper()
	sub, err := conn.SubscribeSync(subject)
	if err != nil {
		t.Logf("subscribe error: %v", err)
		return false
	}
	defer func() { _ = sub.Unsubscribe() }()

	payload := []byte("ping")
	if err := conn.Publish(subject, payload); err != nil {
		t.Logf("publish error: %v", err)
		return false
	}
	msg, err := sub.NextMsg(5 * time.Second)
	if err != nil {
		t.Logf("next-msg error: %v", err)
		return false
	}
	return string(msg.Data) == string(payload)
}
