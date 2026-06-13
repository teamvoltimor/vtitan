package compute

type ComputeState string

const (
	StateReady ComputeState = "ready"
	StateError ComputeState = "error"
)

func (s ComputeState) String() string {
	return string(s)
}
