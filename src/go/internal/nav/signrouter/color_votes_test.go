package signrouter

import "testing"

// An exact tie goes to the color voted first, whichever it is, every time:
// ranging over the map made it a coin flip per execution.
func TestColorVotes_TieGoesToTheFirstVoted(t *testing.T) {
	t.Parallel()

	for range 200 {
		var redFirst, greenFirst colorVotes
		redFirst.add(SignColorRed, 3.6)
		redFirst.add(SignColorGreen, 3.6)
		greenFirst.add(SignColorGreen, 3.6)
		greenFirst.add(SignColorRed, 3.6)
		if got := redFirst.argmax(SignColorUnknown); got != SignColorRed {
			t.Fatalf("red voted first: argmax = %v, want red", got)
		}
		if got := greenFirst.argmax(SignColorUnknown); got != SignColorGreen {
			t.Fatalf("green voted first: argmax = %v, want green", got)
		}
	}
}

func TestColorVotes_LargestTallyWinsAndEmptyFallsBack(t *testing.T) {
	t.Parallel()

	var v colorVotes
	if got := v.argmax(SignColorUnknown); got != SignColorUnknown {
		t.Errorf("empty argmax = %v, want the fallback", got)
	}
	v.add(SignColorRed, 0.5)
	v.add(SignColorGreen, 0.4)
	v.add(SignColorGreen, 0.4)
	if got := v.argmax(SignColorUnknown); got != SignColorGreen {
		t.Errorf("argmax = %v, want green (0.8 against 0.5)", got)
	}
}
