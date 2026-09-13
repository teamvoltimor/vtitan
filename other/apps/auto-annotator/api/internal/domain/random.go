package domain

import (
	"crypto/rand"
	"encoding/hex"
)

// RandomHexString returns a cryptographically random hex string of length
// 2*RandomHexBytes (32 chars). If rand.Read fails, fallback is returned.
func RandomHexString(fallback string) string {
	var b [RandomHexBytes]byte
	if _, err := rand.Read(b[:]); err != nil {
		return fallback
	}
	return hex.EncodeToString(b[:])
}
