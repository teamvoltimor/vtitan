import type { ReactNode } from 'react';

interface AsyncStateProps {
  loading: boolean;
  error: string | null;
  children: ReactNode;
  /** Optional extra actions rendered next to Retry on the error screen. */
  errorActions?: ReactNode;
  onRetry?: () => void;
  loadingLabel?: string;
}

/**
 * Standard loading / error / content gate. Renders a styled spinner while
 * loading, an error panel with a Retry button (plus any extra actions) on
 * failure, and otherwise the children.
 */
export function AsyncState({
  loading,
  error,
  children,
  errorActions,
  onRetry,
  loadingLabel = 'Loading telemetry…',
}: AsyncStateProps) {
  if (error) {
    return (
      <div className="shell-loading" role="alert">
        <p>{error}</p>
        <div className="shell-loading-actions">
          {onRetry && (
            <button type="button" onClick={onRetry}>
              Retry
            </button>
          )}
          {errorActions}
        </div>
      </div>
    );
  }

  if (loading) {
    return (
      <div className="shell-loading" role="status" aria-live="polite">
        <span className="spinner" aria-hidden="true" />
        <span>{loadingLabel}</span>
      </div>
    );
  }

  return <>{children}</>;
}
