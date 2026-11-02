---
title: 'No process observability yet: `internal/telemetry` is a **domain** package (it aggregates LIDAR, IMU and'
priority: p3
section: vtitan
topics:
    - vtitan
---

vision detections for the OLED/UI summary), not logging or metrics, so there is nothing to delete here. The repo has no Prometheus usage and configures `slog` by hand in each `cmd/*`. When the `obs` kit exists (platform queue R152) this repo is a new adopter, not a copy to remove
