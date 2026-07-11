// Command healthcheck is a tiny HTTP readiness probe for the distroless API
// image, which ships no shell and therefore can't run curl/wget.
package main

import (
	"fmt"
	"net/http"
	"os"
	"time"
)

func main() {
	port := os.Getenv("API_PORT")
	if port == "" {
		port = "8000"
	}

	client := http.Client{Timeout: 3 * time.Second}
	resp, err := client.Get(fmt.Sprintf("http://localhost:%s/readyz", port))
	if err != nil {
		os.Exit(1)
	}
	defer resp.Body.Close()

	if resp.StatusCode != http.StatusOK {
		os.Exit(1)
	}
}
