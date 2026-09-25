package nats

import (
	"errors"
	"fmt"
	"hash/fnv"
	"math/rand/v2"
	"slices"
	"strconv"
	"strings"
	"sync"
	"time"

	"github.com/nats-io/nats.go"
	"google.golang.org/protobuf/proto"
	"google.golang.org/protobuf/reflect/protoreflect"
)

// Faults is a fault plan for the messages a process receives: what to do to
// each subject between nats-server and Subscriber.Read. It exists to test
// and simulate a robot whose messages arrive late, lost, stuck or noisy
// without touching the publishers. The zero value injects nothing.
//
// Faults act on the receiving side only, so each subscriber sees its own
// faults: two nodes subscribed to the same subject are affected
// independently, as two consumers of a congested link would be.
type Faults struct {
	// Subjects maps an exact subject (no wildcards) to its faults.
	Subjects map[string]SubjectFaults
	// Seed makes every random draw repeatable. Each subject draws from its
	// own stream derived from Seed and the subject, so adding a fault to
	// one subject does not change the draws of another.
	Seed uint64
}

// SubjectFaults is what happens to the messages of one subject. The zero
// value passes every message through untouched. Faults apply in this order:
// loss, freeze, reorder, delay, then noise at Read.
type SubjectFaults struct {
	// NoiseFields restricts NoiseSigma to fields with these names, at any
	// depth. Empty applies it to every float and double field.
	NoiseFields []string
	// Delay holds every message this long before Read returns it.
	Delay time.Duration
	// Jitter adds a further uniform delay in [0, Jitter] per message.
	// Delivery order is kept; only ReorderRate reorders.
	Jitter time.Duration
	// FreezeDuration is how long each freeze lasts.
	FreezeDuration time.Duration
	// DropRate is the long-run fraction of messages lost, in [0, 1).
	// Losses come in bursts (a Gilbert-Elliott two-state model, the same
	// as pkg/boardsim): see DropBurst.
	DropRate float64
	// DropBurst is the mean number of consecutive messages lost once a
	// loss starts, at least 1 when DropRate is set. 1 gives independent
	// losses.
	DropBurst float64
	// FreezeRate is how many freezes start per second on average (a
	// Poisson process). While frozen, every message that arrives is
	// replaced by the last one let through, as a stuck driver that keeps
	// republishing its last reading would. Zero disables freezes.
	FreezeRate float64
	// ReorderRate is the probability that a message is held back and
	// delivered right after the next one, in [0, 1]. A message held when
	// the publisher stops is never delivered.
	ReorderRate float64
	// NoiseSigma is the standard deviation of zero-mean Gaussian noise
	// added to float and double fields at Read, in each field's own units.
	// Only fields set in the message are touched: proto3 does not
	// distinguish an unset scalar from a zero one.
	NoiseSigma float64
}

// FaultStats counts what a Subscriber's faults did, so a test can prove a
// fault actually happened.
type FaultStats struct {
	// Received counts messages that arrived from nats-server.
	Received uint64
	// Dropped counts messages lost.
	Dropped uint64
	// Frozen counts messages replaced by the last one let through.
	Frozen uint64
	// Reordered counts messages held back behind the next one.
	Reordered uint64
	// Freezes counts freeze windows started.
	Freezes uint64
}

// pending is a received message waiting for its delivery time.
type pending struct {
	due  time.Time
	data []byte
}

// faultQueue applies one subject's faults. Messages arrive from nats.go's
// callback goroutine and leave through Read; mu guards everything below it.
type faultQueue struct {
	// ready is signaled (non-blocking, capacity 1) when a message is queued.
	ready chan struct{}
	now   func() time.Time
	rand  *rand.Rand
	// queue is in delivery order; dues never decrease.
	queue []pending
	// held is a message waiting to be delivered behind the next one.
	held []byte
	// last is the most recent message let through, repeated while frozen.
	last        []byte
	lastDue     time.Time
	frozenUntil time.Time
	nextFreeze  time.Time
	cfg         SubjectFaults
	stats       FaultStats
	mu          sync.Mutex
	// bad is the Gilbert-Elliott state: while bad, every message is lost.
	bad bool
}

// plans holds the fault plan of each connection opened by Connect with one,
// so NewSubscriber can find it without every caller passing it again. The
// entry is removed when the connection closes.
var plans sync.Map // *nats.Conn -> Faults

// errReadDone marks a Read ended by its context; Read wraps the context's
// own error instead.
var errReadDone = errors.New("read done")

// Validate reports a negative duration, a rate outside its range, or a
// subject with a wildcard.
func (f Faults) Validate() error {
	var errs []error
	for subject, sf := range f.Subjects {
		if subject == "" || slices.ContainsFunc([]byte(subject), isWildcard) {
			errs = append(errs, fmt.Errorf("subject %q must be exact, not empty or a wildcard", subject))
			continue
		}
		if err := sf.validate(); err != nil {
			errs = append(errs, fmt.Errorf("subject %s: %w", subject, err))
		}
	}
	if err := errors.Join(errs...); err != nil {
		return fmt.Errorf("nats: faults: %w", err)
	}
	return nil
}

func isWildcard(c byte) bool {
	return c == '*' || c == '>'
}

// Metadata describes the plan as flat key/value text, for a run's
// recording: "seed", and one entry per subject listing its nonzero faults.
// Nil when the plan injects nothing.
func (f Faults) Metadata() map[string]string {
	if !f.active() {
		return nil
	}
	out := map[string]string{"seed": strconv.FormatUint(f.Seed, 10)}
	for subject, sf := range f.Subjects {
		out[subject] = sf.String()
	}
	return out
}

// String lists the nonzero faults, "none" when there are none.
func (sf SubjectFaults) String() string {
	var parts []string
	add := func(on bool, format string, args ...any) {
		if on {
			parts = append(parts, fmt.Sprintf(format, args...))
		}
	}
	add(sf.Delay > 0, "delay=%v", sf.Delay)
	add(sf.Jitter > 0, "jitter=%v", sf.Jitter)
	add(sf.DropRate > 0, "drop=%g burst=%g", sf.DropRate, sf.DropBurst)
	add(sf.FreezeRate > 0, "freeze=%gHz for %v", sf.FreezeRate, sf.FreezeDuration)
	add(sf.ReorderRate > 0, "reorder=%g", sf.ReorderRate)
	add(sf.NoiseSigma > 0, "noise=%g fields=%v", sf.NoiseSigma, sf.NoiseFields)
	if len(parts) == 0 {
		return "none"
	}
	return strings.Join(parts, " ")
}

func (sf SubjectFaults) validate() error {
	switch {
	case sf.Delay < 0 || sf.Jitter < 0:
		return fmt.Errorf("delay %v and jitter %v must not be negative", sf.Delay, sf.Jitter)
	case sf.DropRate < 0 || sf.DropRate >= 1:
		return fmt.Errorf("drop rate %v is outside [0, 1)", sf.DropRate)
	case sf.DropRate > 0 && sf.DropBurst < 1:
		return fmt.Errorf("drop burst %v must be at least 1 when drop rate is set", sf.DropBurst)
	case sf.FreezeRate < 0 || sf.FreezeDuration < 0:
		return fmt.Errorf("freeze rate %v and duration %v must not be negative", sf.FreezeRate, sf.FreezeDuration)
	case sf.ReorderRate < 0 || sf.ReorderRate > 1:
		return fmt.Errorf("reorder rate %v is outside [0, 1]", sf.ReorderRate)
	case sf.NoiseSigma < 0:
		return fmt.Errorf("noise sigma %v must not be negative", sf.NoiseSigma)
	}
	return nil
}

// active reports whether the plan does anything at all, so a connection
// with an empty plan keeps the plain synchronous subscription.
func (f Faults) active() bool {
	return len(f.Subjects) > 0
}

// faultsFor returns the faults conn's plan sets for subject, and whether
// there are any.
func faultsFor(conn *nats.Conn, subject string) (SubjectFaults, uint64, bool) {
	v, ok := plans.Load(conn)
	if !ok {
		return SubjectFaults{}, 0, false
	}
	plan, ok := v.(Faults)
	if !ok {
		return SubjectFaults{}, 0, false
	}
	sf, ok := plan.Subjects[subject]
	return sf, plan.Seed, ok
}

func newFaultQueue(subject string, sf SubjectFaults, seed uint64) *faultQueue {
	h := fnv.New64a()
	_, _ = h.Write([]byte(subject))
	return &faultQueue{
		ready: make(chan struct{}, 1),
		now:   time.Now,
		rand:  rand.New(rand.NewPCG(seed, h.Sum64())),
		cfg:   sf,
	}
}

// receive is the nats.go message handler.
func (q *faultQueue) receive(msg *nats.Msg) {
	q.mu.Lock()
	defer q.mu.Unlock()

	now := q.now()
	q.stats.Received++
	if q.lose() {
		q.stats.Dropped++
		return
	}

	data := msg.Data
	q.advanceFreezes(now)
	if now.Before(q.frozenUntil) {
		if q.last == nil {
			// Frozen before anything got through: there is nothing to
			// repeat, so the message is simply not seen.
			q.stats.Dropped++
			return
		}
		q.stats.Frozen++
		data = q.last
	}
	q.last = data

	if q.held == nil && q.cfg.ReorderRate > 0 && q.rand.Float64() < q.cfg.ReorderRate {
		q.stats.Reordered++
		q.held = data
		return
	}
	q.enqueue(now, data)
	if q.held != nil {
		q.enqueue(now, q.held)
		q.held = nil
	}

	select {
	case q.ready <- struct{}{}:
	default:
	}
}

// enqueue schedules data after the delay and jitter, never ahead of a
// message already queued.
func (q *faultQueue) enqueue(now time.Time, data []byte) {
	delay := q.cfg.Delay
	if q.cfg.Jitter > 0 {
		delay += time.Duration(q.rand.Int64N(int64(q.cfg.Jitter) + 1))
	}
	due := now.Add(delay)
	if due.Before(q.lastDue) {
		due = q.lastDue
	}
	q.lastDue = due
	q.queue = append(q.queue, pending{due: due, data: data})
}

// lose steps the Gilbert-Elliott chain once, as pkg/boardsim's path.lose
// does: the chain leaves bad with probability 1/DropBurst and enters it with
// the probability that keeps the long-run share at DropRate.
func (q *faultQueue) lose() bool {
	if q.cfg.DropRate <= 0 {
		return false
	}
	leave := 1 / q.cfg.DropBurst
	enter := q.cfg.DropRate * leave / (1 - q.cfg.DropRate)
	if q.bad {
		q.bad = q.rand.Float64() >= leave
	} else {
		q.bad = q.rand.Float64() < enter
	}
	return q.bad
}

// advanceFreezes starts every freeze scheduled up to now. Freezes are only
// noticed when messages arrive, which is when they matter.
func (q *faultQueue) advanceFreezes(now time.Time) {
	if q.cfg.FreezeRate <= 0 || q.cfg.FreezeDuration <= 0 {
		return
	}
	if q.nextFreeze.IsZero() {
		q.nextFreeze = now.Add(q.freezeGap())
	}
	for !now.Before(q.nextFreeze) {
		if end := q.nextFreeze.Add(q.cfg.FreezeDuration); end.After(q.frozenUntil) {
			q.frozenUntil = end
		}
		q.stats.Freezes++
		q.nextFreeze = q.nextFreeze.Add(q.cfg.FreezeDuration + q.freezeGap())
	}
}

// freezeGap draws the exponential time to the next freeze.
func (q *faultQueue) freezeGap() time.Duration {
	return time.Duration(q.rand.ExpFloat64() / q.cfg.FreezeRate * float64(time.Second))
}

// next blocks until the head of the queue is due, or done is closed.
func (q *faultQueue) next(done <-chan struct{}) ([]byte, error) {
	for {
		q.mu.Lock()
		var wait time.Duration
		if len(q.queue) > 0 {
			head := q.queue[0]
			wait = head.due.Sub(q.now())
			if wait <= 0 {
				q.queue = q.queue[1:]
				q.mu.Unlock()
				return head.data, nil
			}
		}
		q.mu.Unlock()

		if err := q.await(done, wait); err != nil {
			return nil, err
		}
	}
}

// await blocks until a message is queued, wait elapses (when positive), or
// done is closed.
func (q *faultQueue) await(done <-chan struct{}, wait time.Duration) error {
	var timer <-chan time.Time
	if wait > 0 {
		t := time.NewTimer(wait)
		defer t.Stop()
		timer = t.C
	}
	select {
	case <-done:
		return errReadDone
	case <-q.ready:
	case <-timer:
	}
	return nil
}

// addNoise perturbs msg's float fields in place.
func (q *faultQueue) addNoise(msg proto.Message) {
	if q.cfg.NoiseSigma <= 0 {
		return
	}
	q.mu.Lock()
	defer q.mu.Unlock()
	q.noise(msg.ProtoReflect())
}

func (q *faultQueue) noise(m protoreflect.Message) {
	m.Range(func(fd protoreflect.FieldDescriptor, v protoreflect.Value) bool {
		switch {
		case fd.IsMap():
			return true
		case fd.Kind() == protoreflect.MessageKind || fd.Kind() == protoreflect.GroupKind:
			if fd.IsList() {
				list := v.List()
				for i := range list.Len() {
					q.noise(list.Get(i).Message())
				}
			} else {
				q.noise(v.Message())
			}
			return true
		case !q.noisy(fd):
			return true
		case fd.IsList():
			list := v.List()
			for i := range list.Len() {
				list.Set(i, q.perturb(fd, list.Get(i)))
			}
		default:
			m.Set(fd, q.perturb(fd, v))
		}
		return true
	})
}

// noisy reports whether fd is a float field NoiseFields selects.
func (q *faultQueue) noisy(fd protoreflect.FieldDescriptor) bool {
	if fd.Kind() != protoreflect.FloatKind && fd.Kind() != protoreflect.DoubleKind {
		return false
	}
	return len(q.cfg.NoiseFields) == 0 || slices.Contains(q.cfg.NoiseFields, string(fd.Name()))
}

func (q *faultQueue) perturb(fd protoreflect.FieldDescriptor, v protoreflect.Value) protoreflect.Value {
	n := q.rand.NormFloat64() * q.cfg.NoiseSigma
	if fd.Kind() == protoreflect.FloatKind {
		return protoreflect.ValueOfFloat32(float32(v.Float() + n))
	}
	return protoreflect.ValueOfFloat64(v.Float() + n)
}

func (q *faultQueue) snapshot() FaultStats {
	q.mu.Lock()
	defer q.mu.Unlock()
	return q.stats
}
