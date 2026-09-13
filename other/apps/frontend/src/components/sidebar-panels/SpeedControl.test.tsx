/**
 * src/components/sidebar-panels/SpeedControl.test.tsx
 *
 * The slider readout is the *configured* target speed, not the measured
 * `snapshot.metrics.speed` — these tests lock that in, plus the debounced
 * commit, the rollback on a rejected write, and the disabled states.
 */

import { act, fireEvent, render, screen } from '@testing-library/react';
import { afterEach, describe, expect, it, vi } from 'vitest';
import { SPEED_CONTROL_CONFIG, SPEED_CONTROL_DEBOUNCE_MS, UI_STRINGS } from '../../config';
import type { RobotSnapshot } from '../../types';
import { NodeHealth } from '../../types';
import { SpeedControl } from './SpeedControl';

const mockTelemetry = vi.hoisted(() => {
  const updateSpeed = vi.fn(async () => {});
  return {
    updateSpeed,
    value: {
      snapshot: null as RobotSnapshot | null,
      liveMode: true,
      updateSpeed,
    },
  };
});

vi.mock('../../contexts/telemetryState', () => ({
  useTelemetry: () => mockTelemetry.value,
}));

function makeSnapshot(overrides: Partial<RobotSnapshot> = {}): RobotSnapshot {
  return {
    timestamp: '2026-08-01T12:00:00.000Z',
    mission_name: 'test',
    lidar_points: [],
    path_history: [],
    logs: [],
    metrics: {
      timestamp: '2026-08-01T12:00:00.000Z',
      node_health: NodeHealth.NOMINAL,
      stage: 'ready',
      lidar_available: false,
      imu_available: false,
      camera_available: false,
      odometry_available: false,
    },
    ...overrides,
  };
}

describe('SpeedControl', () => {
  afterEach(() => {
    vi.useRealTimers();
    vi.clearAllMocks();
  });

  it('renders nothing before a snapshot arrives', () => {
    mockTelemetry.value.snapshot = null;
    const { container } = render(<SpeedControl />);
    expect(container.firstChild).toBeNull();
  });

  it('renders the target-speed slider with config bounds and readout', () => {
    mockTelemetry.value.snapshot = makeSnapshot();
    mockTelemetry.value.liveMode = true;
    render(<SpeedControl />);

    const slider = screen.getByRole('slider', {
      name: UI_STRINGS.SPEED_CONTROL,
    }) as HTMLInputElement;
    expect(slider.min).toBe(String(SPEED_CONTROL_CONFIG.MIN));
    expect(slider.max).toBe(String(SPEED_CONTROL_CONFIG.MAX));
    expect(slider.step).toBe(String(SPEED_CONTROL_CONFIG.STEP));
    expect(slider.disabled).toBe(false);
    expect(screen.getByText('1.00 m/s')).toBeTruthy();
  });

  it('disables the slider when not live', () => {
    mockTelemetry.value.snapshot = makeSnapshot();
    mockTelemetry.value.liveMode = false;
    render(<SpeedControl />);
    expect((screen.getByRole('slider') as HTMLInputElement).disabled).toBe(true);
  });

  it('disables the slider when node health is not nominal', () => {
    mockTelemetry.value.snapshot = makeSnapshot({
      metrics: {
        timestamp: '2026-08-01T12:00:00.000Z',
        node_health: NodeHealth.WATCHDOG,
        stage: 'ready',
        lidar_available: false,
        imu_available: false,
        camera_available: false,
        odometry_available: false,
      },
    });
    mockTelemetry.value.liveMode = true;
    render(<SpeedControl />);
    expect((screen.getByRole('slider') as HTMLInputElement).disabled).toBe(true);
  });

  it('debounces the drag and commits the value through updateSpeed', async () => {
    vi.useFakeTimers();
    mockTelemetry.value.snapshot = makeSnapshot();
    mockTelemetry.value.liveMode = true;
    render(<SpeedControl />);

    fireEvent.change(screen.getByRole('slider'), { target: { value: '1.5' } });
    expect(mockTelemetry.value.updateSpeed).not.toHaveBeenCalled();

    await act(async () => {
      vi.advanceTimersByTime(SPEED_CONTROL_DEBOUNCE_MS);
    });

    expect(mockTelemetry.value.updateSpeed).toHaveBeenCalledTimes(1);
    expect(mockTelemetry.value.updateSpeed).toHaveBeenCalledWith(1.5);
    expect(screen.getByText('1.50 m/s')).toBeTruthy();
  });

  it('rolls the slider back and reports when the write is rejected', async () => {
    vi.useFakeTimers();
    mockTelemetry.value.snapshot = makeSnapshot();
    mockTelemetry.value.liveMode = true;
    render(<SpeedControl />);

    mockTelemetry.value.updateSpeed.mockRejectedValueOnce(new Error('backend down'));
    fireEvent.change(screen.getByRole('slider'), { target: { value: '1.5' } });

    await act(async () => {
      vi.advanceTimersByTime(SPEED_CONTROL_DEBOUNCE_MS);
    });

    expect(screen.getByText('Failed to update speed — reverted')).toBeTruthy();
    expect((screen.getByRole('slider') as HTMLInputElement).value).toBe('1');
    expect(screen.getByText('1.00 m/s')).toBeTruthy();
  });
});
