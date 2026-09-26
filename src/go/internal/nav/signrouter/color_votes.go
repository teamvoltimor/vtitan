package signrouter

// colorVotes is a confidence-weighted color tally that remembers the order
// colors first received a vote, so an exact tie goes to the color voted
// first, as Python's max over an insertion-ordered dict does. Ranging over
// a Go map instead broke ties by the map's random iteration order: one
// red and one green look of equal confidence made the same run publish a
// different sign color from one execution to the next.
type colorVotes struct {
	byColor map[SignColor]float64
	order   []SignColor
}

// add folds one observation's confidence into its color's tally.
func (v *colorVotes) add(color SignColor, confidence float64) {
	if v.byColor == nil {
		v.byColor = map[SignColor]float64{}
	}
	if _, seen := v.byColor[color]; !seen {
		v.order = append(v.order, color)
	}
	v.byColor[color] += confidence
}

// argmax is the color with the largest tally, the earliest voted on a tie,
// and fallback with no votes at all.
func (v *colorVotes) argmax(fallback SignColor) SignColor {
	best := fallback
	for i, color := range v.order {
		if i == 0 || v.byColor[color] > v.byColor[best] {
			best = color
		}
	}
	return best
}
