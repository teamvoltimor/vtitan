/**
 * src/components/visualizers/VisionStrip.test.tsx
 *
 * Audit §7.2 regression guard: the snapshot's vision_detections must render
 * as bboxes (coloured by class) and stay hidden when the frame is empty.
 */

import { render } from '@testing-library/react';
import { describe, expect, it } from 'vitest';
import { VISION_CONFIG } from '../../config';
import type { Detection } from '../../types';
import { VisionStrip } from './VisionStrip';

const detections: Detection[] = [
  { class_name: 'red_cone', confidence: 0.92, bbox_x: 10, bbox_y: 20, bbox_w: 60, bbox_h: 40 },
  { class_name: 'green_cone', confidence: 0.61, bbox_x: 200, bbox_y: 300, bbox_w: 80, bbox_h: 90 },
];

describe('VisionStrip', () => {
  it('renders nothing when there are no detections', () => {
    const { container } = render(<VisionStrip detections={[]} />);
    expect(container.firstChild).toBeNull();
  });

  it('draws a bbox and label per detection', () => {
    const { container } = render(<VisionStrip detections={detections} />);
    expect(container.querySelectorAll('svg rect').length).toBe(4); // 2 bboxes + 2 label chips
    expect(container.querySelector('svg')?.getAttribute('viewBox')).toBe(VISION_CONFIG.VIEWBOX);
    expect(container.querySelectorAll('svg text').length).toBe(2);
  });

  it('colours red-detections with the danger colour and labels confidence', () => {
    const { container } = render(<VisionStrip detections={detections} />);
    const text = container.querySelector('svg text');
    expect(text?.textContent).toContain('red_cone');
    expect(text?.textContent).toContain('92 %');
    expect(text?.getAttribute('fill')).toBe('#ffffff');
  });

  it('labels the strip with the detection count for screen readers', () => {
    const { getByRole } = render(<VisionStrip detections={detections} />);
    expect(getByRole('img').getAttribute('aria-label')).toBe('2 vision detection(s)');
  });
});
