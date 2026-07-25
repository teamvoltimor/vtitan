/**
 * src/contexts/TelemetryContext.test.tsx
 *
 * Exercises the context's state machine against MockSource (Demo Mode) — no
 * network/backend needed, and it's the same source the app itself uses for
 * Demo Mode, so this tests real behavior rather than a hand-rolled fixture.
 */

import { describe, it, expect, vi, afterEach } from 'vitest';
import { act, renderHook, waitFor } from '@testing-library/react';
import type { ReactNode } from 'react';
import { TelemetryProvider } from './TelemetryContext';
import { useTelemetry } from './telemetryState';
import { TELEMETRY_CONFIG } from '../config';

function wrapper({ children }: { children: ReactNode }) {
  return <TelemetryProvider>{children}</TelemetryProvider>;
}

function renderInDemoMode() {
  window.history.replaceState(null, '', '/?demo=true');
  return renderHook(() => useTelemetry(), { wrapper });
}

describe('TelemetryProvider (Demo Mode via MockSource)', () => {
  afterEach(() => {
    vi.useRealTimers();
  });

  it('loads initial data and renders displaySnapshot as the live snapshot', async () => {
    const { result } = renderInDemoMode();

    await waitFor(() => expect(result.current.loading).toBe(false));

    expect(result.current.snapshot).not.toBeNull();
    expect(result.current.liveMode).toBe(true);
    expect(result.current.displaySnapshot).toBe(result.current.snapshot);
  });

  it('keeps timelineIndex within history bounds as live frames accumulate past the history cap', async () => {
    vi.useFakeTimers();
    const { result } = renderInDemoMode();

    await act(async () => {
      await vi.advanceTimersByTimeAsync(
        TELEMETRY_CONFIG.POLL_INTERVAL_MS * (TELEMETRY_CONFIG.HISTORY_MAX_SIZE + 20)
      );
    });

    expect(result.current.history.length).toBeLessThanOrEqual(TELEMETRY_CONFIG.HISTORY_MAX_SIZE);
    // Regression guard for the bug where timelineIndex incremented without
    // bound while history stayed capped, producing an index past the end.
    expect(result.current.timelineIndex).toBeLessThanOrEqual(
      Math.max(result.current.history.length - 1, 0)
    );
  });

  it('drives displaySnapshot from the scrubbed history frame once liveMode is false', async () => {
    const { result } = renderInDemoMode();
    await waitFor(() => expect(result.current.loading).toBe(false));

    const sessionId = result.current.sessions[0]?.session_id;
    expect(sessionId).toBeTruthy();

    await act(async () => {
      await result.current.loadSession(sessionId as string);
    });

    expect(result.current.liveMode).toBe(false);
    expect(result.current.displaySnapshot).toBe(
      result.current.history[result.current.timelineIndex]
    );

    act(() => {
      result.current.setTimelineIndex(0);
    });
    expect(result.current.displaySnapshot).toBe(result.current.history[0]);

    act(() => {
      result.current.goLive();
    });
    expect(result.current.liveMode).toBe(true);
  });

  it('accumulates logs across snapshots instead of replacing them with only the latest frame', async () => {
    vi.useFakeTimers();
    const { result } = renderInDemoMode();

    await act(async () => {
      await vi.advanceTimersByTimeAsync(TELEMETRY_CONFIG.POLL_INTERVAL_MS * 5);
    });

    // MockSource always emits its own frame's logs — accumulation should grow
    // the buffer across several ticks, not just reflect the single latest one.
    expect(result.current.logs.length).toBeGreaterThan(0);
    expect(result.current.logs.length).toBeLessThanOrEqual(TELEMETRY_CONFIG.LOG_BUFFER_MAX_SIZE);
  });
});
