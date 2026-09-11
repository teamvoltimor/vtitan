/**
 * src/components/visualizers/StateDiagram.test.tsx
 *
 * Regression guard for audit §3.3: the wire carries lowercase RobotState
 * values, so the diagram must normalise casing at the boundary — a mixed-case
 * payload must highlight the right node and never mute the whole diagram.
 */

import { render } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { COLORS } from '../../config';
import type { StringMsg } from '../../types';
import { StateDiagram } from './StateDiagram';

function activeNodeCount(container: HTMLElement): number {
  return Array.from(container.querySelectorAll('circle')).filter(
    (circle) => circle.getAttribute('fill') === COLORS.SUCCESS
  ).length;
}

function titleOf(container: HTMLElement): string | null {
  return container.querySelector('svg title')?.textContent ?? null;
}

function nodeNames(container: HTMLElement): Array<string | null> {
  return Array.from(container.querySelectorAll('svg text')).map((node) => node.textContent);
}

describe('StateDiagram', () => {
  it('highlights the node matching a lowercase payload', () => {
    const { container } = render(<StateDiagram data={{ data: 'racing' }} />);
    expect(titleOf(container)).toBe('Robot state: racing');
    expect(activeNodeCount(container)).toBe(1);
  });

  it('normalises uppercase payloads to the canonical lowercase state (§3.3)', () => {
    const { container } = render(<StateDiagram data={{ data: 'RACING' }} />);
    expect(titleOf(container)).toBe('Robot state: racing');
    expect(activeNodeCount(container)).toBe(1);
  });

  it('renders every state node', () => {
    const { container } = render(<StateDiagram data={{ data: 'ready' }} />);
    expect(nodeNames(container)).toEqual(
      expect.arrayContaining(['boot_check', 'ready', 'racing', 'finished'])
    );
  });

  it('shows an unknown-state notice and highlights nothing for unrecognised values', () => {
    const { container } = render(<StateDiagram data={{ data: 'calibrating' }} />);
    expect(titleOf(container)).toBe('Robot state: calibrating');
    expect(activeNodeCount(container)).toBe(0);
    expect(container.textContent).toContain('Unrecognized state: calibrating');
  });

  it('falls back to "unknown" when the payload is missing or empty', () => {
    const { container } = render(<StateDiagram data={undefined as unknown as StringMsg} />);
    expect(titleOf(container)).toBe('Robot state: unknown');
    expect(activeNodeCount(container)).toBe(0);
  });
});
