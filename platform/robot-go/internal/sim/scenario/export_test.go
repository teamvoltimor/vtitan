package scenario

// ParseResultForTest exposes the unexported parseResult to this package's
// black-box (scenario_test) tests, without making JSON parsing part of the
// public API — result_test.go is the only caller.
func ParseResultForTest(data string) (Result, error) {
	return parseResult([]byte(data))
}
