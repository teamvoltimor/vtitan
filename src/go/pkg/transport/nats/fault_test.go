package nats_test

import (
	"context"
	"errors"
	"maps"
	"slices"
	"testing"
	"time"

	"github.com/teamvoltimor/vtitan/src/go/pkg/transport/nats"

	actuationv1 "github.com/teamvoltimor/vtitan/src/go/internal/schema/pb/vtitan/actuation/v1"
)

const (
	faultSubject = "vtitan.test.v1.faults"
	// drainWait is how long drain waits for one more message before
	// deciding the stream has ended.
	drainWait = 200 * time.Millisecond
)

// faultyPair connects with faults on faultSubject and returns a subscriber
// and a publisher on it.
func faultyPair(
	t *testing.T, sf nats.SubjectFaults, seed uint64,
) (*nats.Subscriber[*actuationv1.AckermannCmd], *nats.Publisher[*actuationv1.AckermannCmd]) {
	t.Helper()

	cfg := nats.DefaultConfig(startTestServer(t), t.Name())
	cfg.Faults = nats.Faults{Subjects: map[string]nats.SubjectFaults{faultSubject: sf}, Seed: seed}
	conn, err := nats.Connect(context.Background(), cfg)
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)

	sub, err := nats.NewSubscriber[actuationv1.AckermannCmd](conn, faultSubject)
	if err != nil {
		t.Fatalf("NewSubscriber() error = %v, want nil", err)
	}
	t.Cleanup(func() { _ = sub.Close() })
	return sub, nats.NewPublisher[*actuationv1.AckermannCmd](conn, faultSubject)
}

// publishSeq publishes n commands whose Speed is their sequence number,
// 1 to n, gap apart.
func publishSeq(t *testing.T, pub *nats.Publisher[*actuationv1.AckermannCmd], n int, gap time.Duration) {
	t.Helper()
	for i := 1; i <= n; i++ {
		if err := pub.Publish(&actuationv1.AckermannCmd{Speed: float32(i), SteeringAngle: 0.5}); err != nil {
			t.Fatalf("Publish() error = %v, want nil", err)
		}
		if gap > 0 {
			time.Sleep(gap)
		}
	}
}

// drain reads until no message arrives for drainWait and returns the
// sequence numbers read.
func drain(t *testing.T, sub *nats.Subscriber[*actuationv1.AckermannCmd]) []int {
	t.Helper()
	var got []int
	for {
		ctx, cancel := context.WithTimeout(context.Background(), drainWait)
		msg, err := sub.Read(ctx)
		cancel()
		if errors.Is(err, context.DeadlineExceeded) {
			return got
		}
		if err != nil {
			t.Fatalf("Read() error = %v, want nil", err)
		}
		got = append(got, int(msg.GetSpeed()))
	}
}

func TestSubscriber_NoFaultPlan_HasNoFaultStats(t *testing.T) {
	t.Parallel()

	conn, err := nats.Connect(context.Background(), nats.DefaultConfig(startTestServer(t), t.Name()))
	if err != nil {
		t.Fatalf("Connect() error = %v, want nil", err)
	}
	t.Cleanup(conn.Close)
	sub, err := nats.NewSubscriber[actuationv1.AckermannCmd](conn, faultSubject)
	if err != nil {
		t.Fatalf("NewSubscriber() error = %v, want nil", err)
	}
	t.Cleanup(func() { _ = sub.Close() })

	if _, ok := sub.FaultStats(); ok {
		t.Error("FaultStats() ok = true without a fault plan, want false")
	}
}

func TestFaults_Delay_HoldsEveryMessage(t *testing.T) {
	t.Parallel()

	const delay = 80 * time.Millisecond
	sub, pub := faultyPair(t, nats.SubjectFaults{Delay: delay}, 1)

	start := time.Now()
	publishSeq(t, pub, 1, 0)
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	if _, err := sub.Read(ctx); err != nil {
		t.Fatalf("Read() error = %v, want nil", err)
	}
	if got := time.Since(start); got < delay {
		t.Errorf("Read() returned after %v, want at least the %v delay", got, delay)
	}
}

func TestFaults_Jitter_KeepsOrder(t *testing.T) {
	t.Parallel()

	sub, pub := faultyPair(t, nats.SubjectFaults{Jitter: 20 * time.Millisecond}, 1)
	const n = 50
	publishSeq(t, pub, n, 0)

	got := drain(t, sub)
	if len(got) != n || !slices.IsSorted(got) {
		t.Errorf("read %v, want 1..%d in order", got, n)
	}
}

func TestFaults_Drop_LosesAboutTheRateAndCountsIt(t *testing.T) {
	t.Parallel()

	sub, pub := faultyPair(t, nats.SubjectFaults{DropRate: 0.3, DropBurst: 3}, 7)
	const n = 400
	publishSeq(t, pub, n, 0)

	got := drain(t, sub)
	stats, _ := sub.FaultStats()
	if stats.Received != n || int(stats.Dropped)+len(got) != n {
		t.Fatalf("received %d, dropped %d, read %d, want %d received = dropped + read",
			stats.Received, stats.Dropped, len(got), n)
	}
	// 400 draws from a chain with a 0.3 stationary share: a wide band, as
	// bursts make the share noisy, but far from 0 and from 1.
	if share := float64(stats.Dropped) / n; share < 0.15 || share > 0.45 {
		t.Errorf("dropped share %.2f, want near 0.3", share)
	}
}

func TestFaults_Drop_SameSeedSameLosses(t *testing.T) {
	t.Parallel()

	run := func() []int {
		sub, pub := faultyPair(t, nats.SubjectFaults{DropRate: 0.3, DropBurst: 2}, 42)
		publishSeq(t, pub, 200, 0)
		return drain(t, sub)
	}
	if a, b := run(), run(); !slices.Equal(a, b) {
		t.Errorf("two runs with seed 42 read different messages:\n%v\n%v", a, b)
	}
}

func TestFaults_Freeze_RepeatsTheLastMessage(t *testing.T) {
	t.Parallel()

	sub, pub := faultyPair(t, nats.SubjectFaults{
		FreezeRate:     20,
		FreezeDuration: 30 * time.Millisecond,
	}, 3)
	const n = 150
	publishSeq(t, pub, n, 2*time.Millisecond)

	got := drain(t, sub)
	stats, _ := sub.FaultStats()
	if stats.Freezes == 0 || stats.Frozen == 0 {
		t.Fatalf("freezes %d, frozen messages %d, want both above zero over %d messages",
			stats.Freezes, stats.Frozen, n)
	}
	if len(got) != n || !slices.IsSorted(got) {
		t.Fatalf("read %d messages %v, want %d, never going backwards", len(got), got, n)
	}
	repeats := 0
	for i := 1; i < len(got); i++ {
		if got[i] == got[i-1] {
			repeats++
		}
	}
	if repeats != int(stats.Frozen) {
		t.Errorf("read %d repeats, want one per frozen message (%d)", repeats, stats.Frozen)
	}
}

func TestFaults_Reorder_SwapsWithTheNext(t *testing.T) {
	t.Parallel()

	sub, pub := faultyPair(t, nats.SubjectFaults{ReorderRate: 1}, 1)
	publishSeq(t, pub, 6, 0)

	if got, want := drain(t, sub), []int{2, 1, 4, 3, 6, 5}; !slices.Equal(got, want) {
		t.Errorf("read %v, want %v", got, want)
	}
}

func TestFaults_Noise_TouchesOnlyTheNamedField(t *testing.T) {
	t.Parallel()

	sub, pub := faultyPair(t, nats.SubjectFaults{NoiseSigma: 0.1, NoiseFields: []string{"speed"}}, 1)
	if err := pub.Publish(&actuationv1.AckermannCmd{Speed: 1, SteeringAngle: 0.5}); err != nil {
		t.Fatalf("Publish() error = %v, want nil", err)
	}
	ctx, cancel := context.WithTimeout(context.Background(), time.Second)
	defer cancel()
	msg, err := sub.Read(ctx)
	if err != nil {
		t.Fatalf("Read() error = %v, want nil", err)
	}
	if msg.GetSpeed() == 1 {
		t.Error("speed = 1 exactly, want noise added")
	}
	if msg.GetSteeringAngle() != 0.5 {
		t.Errorf("steering angle = %v, want 0.5 untouched", msg.GetSteeringAngle())
	}
}

func TestFaults_Validate(t *testing.T) {
	t.Parallel()

	tests := []struct {
		name    string
		subject string
		sf      nats.SubjectFaults
		wantErr bool
	}{
		{name: "zero is valid", subject: "a.b", wantErr: false},
		{name: "wildcard subject", subject: "a.*", wantErr: true},
		{name: "tail wildcard subject", subject: "a.>", wantErr: true},
		{name: "empty subject", subject: "", wantErr: true},
		{name: "negative delay", subject: "a.b", sf: nats.SubjectFaults{Delay: -1}, wantErr: true},
		{name: "drop rate one", subject: "a.b", sf: nats.SubjectFaults{DropRate: 1, DropBurst: 1}, wantErr: true},
		{name: "drop without burst", subject: "a.b", sf: nats.SubjectFaults{DropRate: 0.1}, wantErr: true},
		{name: "reorder above one", subject: "a.b", sf: nats.SubjectFaults{ReorderRate: 1.5}, wantErr: true},
		{name: "negative freeze", subject: "a.b", sf: nats.SubjectFaults{FreezeRate: -1}, wantErr: true},
		{name: "negative noise", subject: "a.b", sf: nats.SubjectFaults{NoiseSigma: -0.1}, wantErr: true},
	}
	for _, tt := range tests {
		t.Run(tt.name, func(t *testing.T) {
			t.Parallel()
			f := nats.Faults{Subjects: map[string]nats.SubjectFaults{tt.subject: tt.sf}}
			if err := f.Validate(); (err != nil) != tt.wantErr {
				t.Errorf("Validate() error = %v, wantErr %v", err, tt.wantErr)
			}
		})
	}
}

func TestConnect_RejectsAnInvalidFaultPlan(t *testing.T) {
	t.Parallel()

	cfg := nats.DefaultConfig(startTestServer(t), t.Name())
	cfg.Faults = nats.Faults{Subjects: map[string]nats.SubjectFaults{"a.*": {}}}
	if conn, err := nats.Connect(context.Background(), cfg); err == nil {
		conn.Close()
		t.Error("Connect() error = nil with a wildcard fault subject, want an error")
	}
}

func TestFaults_Metadata(t *testing.T) {
	t.Parallel()

	if got := (nats.Faults{Seed: 3}).Metadata(); got != nil {
		t.Errorf("Metadata() of an empty plan = %v, want nil", got)
	}
	f := nats.Faults{Seed: 3, Subjects: map[string]nats.SubjectFaults{
		"a.b": {Delay: 40 * time.Millisecond, DropRate: 0.1, DropBurst: 2},
		"c.d": {},
	}}
	want := map[string]string{"seed": "3", "a.b": "delay=40ms drop=0.1 burst=2", "c.d": "none"}
	if got := f.Metadata(); !maps.Equal(got, want) {
		t.Errorf("Metadata() = %v, want %v", got, want)
	}
}
