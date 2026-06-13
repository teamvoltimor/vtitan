package handlers

type ResponseState string

const (
	StateSuccess ResponseState = "success"
	StateError   ResponseState = "error"
)

func (s ResponseState) String() string {
	return string(s)
}
