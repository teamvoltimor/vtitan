import type { ReactNode } from 'react';

/** A label/value row used in the telemetry list (`span` + `strong`). */
export function MetricRow({ label, value }: { label: string; value: ReactNode }) {
  return (
    <div>
      <span>{label}</span>
      <strong>{value}</strong>
    </div>
  );
}

/** A label/value tile used in the metrics grid (`p` + `strong`). */
export function StatTile({ label, value }: { label: string; value: ReactNode }) {
  return (
    <div>
      <p>{label}</p>
      <strong>{value}</strong>
    </div>
  );
}
