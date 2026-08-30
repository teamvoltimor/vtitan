package visionv1

// DetectionsSubject is the NATS subject declared in detections.proto's own
// docstring -- named here, next to the generated message types, so every
// publisher/subscriber pair references one source of truth instead of each
// cmd/* binary retyping the same literal.
const DetectionsSubject = "vtitan.vision.v1.detections"
