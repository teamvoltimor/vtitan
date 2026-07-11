import type { ReactNode } from 'react';

/** Uppercase mono section label used throughout the panels. */
export function Label({ children }: { children: ReactNode }) {
  return <p className="label">{children}</p>;
}
