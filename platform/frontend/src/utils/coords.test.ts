import { describe, it, expect } from 'vitest';
import { simToThree } from './coords';

describe('simToThree', () => {
  it('maps the track centre to the scene origin', () => {
    const [x, y, z] = simToThree({ x: 1.5, y: 1.5, z: 0 });
    expect(x).toBeCloseTo(0);
    expect(y).toBeCloseTo(0);
    expect(z).toBeCloseTo(0);
  });

  it('maps sim-Z to scene-Y (up) and flips sim-Y to scene -Z', () => {
    const [x, y, z] = simToThree({ x: 2.5, y: 0.5, z: 0.3 });
    expect(x).toBeCloseTo(1.0); // 2.5 - 1.5
    expect(y).toBeCloseTo(0.3); // sim z → scene y
    expect(z).toBeCloseTo(1.0); // -(0.5 - 1.5)
  });
});
