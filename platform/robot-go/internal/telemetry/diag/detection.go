package diag

// initialBestScore mirrors _best_detection's own sentinel
// (platform/robot/ros2_ws/src/vtitan_state_machine/vtitan_state_machine/
// telemetry_bridge_node.py): any real detection's confidence*area is >= 0,
// so a strictly-negative sentinel guarantees the first candidate always
// replaces it.
const initialBestScore = -1.0

// bestDetection picks the single most salient detection, ranked by
// confidence*area rather than confidence alone, matching _best_detection:
// a small high-confidence false positive and a large low-confidence smear
// are both less trustworthy than one detection that scores well on both
// axes. Returns ok=false when detections is empty.
func bestDetection(detections []Detection) (best Detection, ok bool) {
	bestScore := initialBestScore
	for _, detection := range detections {
		score := detection.Confidence * detection.Area()
		if score > bestScore {
			bestScore = score
			best = detection
			ok = true
		}
	}
	return best, ok
}

// fillBestDetection writes the highest-scoring detection (see
// bestDetection) into summary, matching `_publish_ui_summary`'s
// `class_id, confidence = detection if detection is not None else (None, None)`.
// Leaves summary at its zero value (HasBestDetection=false) when detections
// is empty, matching the None/None fallback.
func fillBestDetection(summary *TelemetrySummary, detections []Detection) {
	best, ok := bestDetection(detections)
	if !ok {
		return
	}
	summary.BestDetectionClassID = best.ClassName
	summary.BestDetectionConfidence = best.Confidence
	summary.HasBestDetection = true
}
