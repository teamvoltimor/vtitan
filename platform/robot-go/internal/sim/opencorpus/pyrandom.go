package opencorpus

// A faithful reimplementation of the slice of CPython's `random` module that
// balanced_128_cases depends on: MT19937 seeded the way random.Random(int)
// seeds it, and random.shuffle's Fisher-Yates using _randbelow.
//
// # Why not just use math/rand
//
// Because the corpus is named by its seed. "balanced128 at seed 0" has to
// mean the SAME 128 scenarios in Go as in Python, or two arms of an A/B run
// in different languages are comparing different corpora while both claiming
// the seed as evidence they are comparable. Go's own shuffle would produce a
// perfectly good corpus that silently is not that one.
//
// This is deliberately the only place in the tree that reimplements a
// language runtime, and it exists solely to make a seed portable. It is
// pinned against a golden dumped from Python at four seeds; see
// TestBalanced128_MatchesPython.

// mt19937 is the Mersenne Twister CPython's random module is built on.
const (
	mtN         = 624
	mtM         = 397
	mtMatrixA   = 0x9908b0df
	mtUpperMask = 0x80000000
	mtLowerMask = 0x7fffffff
)

type mt19937 struct {
	state [mtN]uint32
	index int
}

// initGenrand is CPython's init_genrand: the scalar seeding routine, used
// here only as the first half of initByArray.
func (m *mt19937) initGenrand(seed uint32) {
	m.state[0] = seed
	for i := 1; i < mtN; i++ {
		prev := m.state[i-1]
		m.state[i] = 1812433253*(prev^(prev>>30)) + uint32(i)
	}
	m.index = mtN
}

// initByArray is CPython's init_by_array, which is what random.seed(int)
// actually calls -- NOT init_genrand. Seeding with the scalar routine
// instead produces a different stream from the same integer, which would
// look like a working implementation right up until the corpora diverged.
func (m *mt19937) initByArray(key []uint32) {
	m.initGenrand(19650218)
	i, j := 1, 0
	k := mtN
	if len(key) > k {
		k = len(key)
	}
	for ; k > 0; k-- {
		prev := m.state[i-1]
		m.state[i] = (m.state[i] ^ ((prev ^ (prev >> 30)) * 1664525)) + key[j] + uint32(j)
		i++
		j++
		if i >= mtN {
			m.state[0] = m.state[mtN-1]
			i = 1
		}
		if j >= len(key) {
			j = 0
		}
	}
	for k = mtN - 1; k > 0; k-- {
		prev := m.state[i-1]
		m.state[i] = (m.state[i] ^ ((prev ^ (prev >> 30)) * 1566083941)) - uint32(i)
		i++
		if i >= mtN {
			m.state[0] = m.state[mtN-1]
			i = 1
		}
	}
	// MSB is 1, assuring a non-zero initial array.
	m.state[0] = mtUpperMask
}

// newPyRandom seeds a generator the way CPython's random.Random(n) does for
// a non-negative int: the absolute value is split into little-endian 32-bit
// words and handed to init_by_array, with zero represented as a single zero
// word rather than an empty key.
func newPyRandom(seed uint64) *mt19937 {
	var key []uint32
	if seed == 0 {
		key = []uint32{0}
	} else {
		for n := seed; n > 0; n >>= 32 {
			key = append(key, uint32(n&0xffffffff))
		}
	}
	m := &mt19937{}
	m.initByArray(key)
	return m
}

// genrandUint32 is CPython's genrand_uint32, including the tempering.
func (m *mt19937) genrandUint32() uint32 {
	if m.index >= mtN {
		for i := range mtN {
			y := (m.state[i] & mtUpperMask) | (m.state[(i+1)%mtN] & mtLowerMask)
			next := m.state[(i+mtM)%mtN] ^ (y >> 1)
			if y&1 != 0 {
				next ^= mtMatrixA
			}
			m.state[i] = next
		}
		m.index = 0
	}
	y := m.state[m.index]
	m.index++
	y ^= y >> 11
	y ^= (y << 7) & 0x9d2c5680
	y ^= (y << 15) & 0xefc60000
	y ^= y >> 18
	return y
}

// getRandBits is random.getrandbits for k <= 32, which is the only range
// _randbelow reaches here (the largest bound is the combo count, 128).
func (m *mt19937) getRandBits(k uint) uint32 {
	if k == 0 {
		return 0
	}
	return m.genrandUint32() >> (32 - k)
}

// randBelow is CPython's _randbelow_with_getrandbits: draw k bits where k is
// n's bit length, and REJECT anything at or above n rather than taking a
// modulus. The rejection loop is what consumes a variable number of draws,
// so a modulus here would desynchronise the stream from Python's after the
// first rejected value even though it returns numbers in the same range.
func (m *mt19937) randBelow(n uint32) uint32 {
	if n == 0 {
		return 0
	}
	k := uint(0)
	for v := n; v > 0; v >>= 1 {
		k++
	}
	r := m.getRandBits(k)
	for r >= n {
		r = m.getRandBits(k)
	}
	return r
}

// shuffle is random.shuffle: Fisher-Yates walking DOWN from the last index,
// which is the direction CPython uses. Walking up draws the same count of
// random numbers and produces a different permutation.
func (m *mt19937) shuffle(order []int) {
	for i := len(order) - 1; i > 0; i-- {
		j := int(m.randBelow(uint32(i + 1)))
		order[i], order[j] = order[j], order[i]
	}
}
