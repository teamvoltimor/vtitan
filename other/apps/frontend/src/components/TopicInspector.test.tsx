/**
 * src/components/TopicInspector.test.tsx
 *
 * List state (filter, expand, visual/JSON mode) and staleness — the parts of
 * the inspector that carry real logic. The per-topic visualizers have their
 * own tests; here we only assert dispatch into the right one.
 */

import { fireEvent, render, screen } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { TELEMETRY_CONFIG, UI_STRINGS } from '../config';
import type { TopicsSnapshot, TopicUpdate } from '../types';
import { RosMessageType } from '../types';
import { TopicInspector } from './TopicInspector';

function makeTopic(overrides: Partial<TopicUpdate> = {}): TopicUpdate {
  return {
    topic_name: '/robot_state',
    message_type: RosMessageType.STRING,
    timestamp: new Date().toISOString(),
    update_rate_hz: 10,
    data: { data: 'racing' },
    ...overrides,
  };
}

function makeTopics(...topics: TopicUpdate[]): TopicsSnapshot {
  return { timestamp: new Date().toISOString(), topics };
}

function expandTopic(topicName: string) {
  const header = screen.getByText(topicName).closest('.topic-header') as HTMLElement;
  fireEvent.click(header);
}

const freshTopics = makeTopics(
  makeTopic(),
  makeTopic({
    topic_name: '/cmd_vel',
    message_type: RosMessageType.TWIST,
    update_rate_hz: 20,
    data: { linear: { x: 0, y: 0, z: 0 }, angular: { x: 0, y: 0, z: 0 } },
  }),
  makeTopic({
    topic_name: '/sensor/scan',
    message_type: RosMessageType.LASER_SCAN,
    update_rate_hz: 0,
    timestamp: new Date(
      Date.now() - (TELEMETRY_CONFIG.TOPIC_STALENESS_THRESHOLD_SECONDS + 5) * 1000
    ).toISOString(),
    data: {
      ranges: [],
      angle_min: 0,
      angle_max: 3.14,
      angle_increment: 0.1,
      range_min: 0.1,
      range_max: 2,
    },
  })
);

describe('TopicInspector', () => {
  it('shows the empty state before topics arrive', () => {
    const { container } = render(<TopicInspector topics={null} />);
    expect(screen.getByText('Waiting for ROS2 topics…')).toBeTruthy();
    expect(screen.getByText('No topics')).toBeTruthy();
    expect(container.querySelector('.topic-list')).toBeNull();
  });

  it('shows the empty state for an empty topic list', () => {
    render(<TopicInspector topics={makeTopics()} />);
    expect(screen.getByText('Waiting for ROS2 topics…')).toBeTruthy();
  });

  it('lists topics with type and formatted rate', () => {
    render(<TopicInspector topics={freshTopics} />);
    expect(screen.getByText('/robot_state')).toBeTruthy();
    expect(screen.getByText(RosMessageType.STRING)).toBeTruthy();
    expect(screen.getByText('10.0 Hz')).toBeTruthy();
    expect(screen.getByText('0.0 Hz')).toBeTruthy();
  });

  it('marks stale topics with the STALE badge', () => {
    render(<TopicInspector topics={freshTopics} />);
    const item = screen.getByText('/sensor/scan').closest('.topic-item');
    expect(item?.querySelector('.stale-badge')).toBeTruthy();
    expect(screen.getByText('STALE')).toBeTruthy();
  });

  it('filters the topic list by search text', () => {
    render(<TopicInspector topics={freshTopics} />);
    fireEvent.change(screen.getByPlaceholderText(UI_STRINGS.FILTER_TOPICS), {
      target: { value: 'cmd_vel' },
    });
    expect(screen.getByText('/cmd_vel')).toBeTruthy();
    expect(screen.queryByText('/robot_state')).toBeNull();
  });

  it('toggles the mode button between visual and JSON', () => {
    render(<TopicInspector topics={freshTopics} />);

    const initial = screen.getByRole('button', { name: 'Show Visual' });
    expect(initial.className).toBe('mode-toggle');

    fireEvent.click(initial);
    const visual = screen.getByRole('button', { name: 'Show JSON' });
    expect(visual.className).toContain('active');

    fireEvent.click(visual);
    expect(screen.getByRole('button', { name: 'Show Visual' }).className).toBe('mode-toggle');
  });

  it('shows the raw JSON view when expanding a topic in default mode', () => {
    render(<TopicInspector topics={freshTopics} />);
    expandTopic('/robot_state');
    expect(screen.getByText('"racing"')).toBeTruthy();
  });

  it('dispatches an expanded topic to its specialised visualizer in visual mode', () => {
    render(<TopicInspector topics={freshTopics} />);
    fireEvent.click(screen.getByRole('button', { name: 'Show Visual' }));
    expandTopic('/robot_state');

    const diagram = document.querySelector('.state-diagram');
    expect(diagram?.querySelector('svg title')?.textContent).toBe('Robot state: racing');
  });

  it('opens a full-view modal from the expand button and closes it', () => {
    render(<TopicInspector topics={freshTopics} />);
    fireEvent.click(screen.getByRole('button', { name: 'Expand /robot_state in full view' }));

    const dialog = screen.getByRole('dialog', { name: '/robot_state' });
    expect(dialog).toBeTruthy();

    fireEvent.click(screen.getByRole('button', { name: 'Close' }));
    expect(screen.queryByRole('dialog')).toBeNull();
  });
});
