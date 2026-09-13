import { describe, expect, it } from 'vitest';
import {
  clamp,
  formatNumber,
  formatTimestamp,
  getErrorMessage,
  radiansToDegrees,
  timestampAgeSeconds,
} from './formatting';

describe('formatNumber', () => {
  it('formats with default 2 decimals', () => {
    expect(formatNumber(1.2345)).toBe('1.23');
  });
  it('appends a unit', () => {
    expect(formatNumber(1.2345, { unit: 'm/s' })).toBe('1.23 m/s');
  });
  it('returns the fallback for nullish input', () => {
    expect(formatNumber(null)).toBe('N/A');
    expect(formatNumber(undefined, { default: '—' })).toBe('—');
  });
});

describe('formatTimestamp', () => {
  it('returns N/A for nullish or unparseable values', () => {
    expect(formatTimestamp(null)).toBe('N/A');
    expect(formatTimestamp('not-a-date')).toBe('N/A');
  });
  it('parses ISO strings and Unix seconds without throwing', () => {
    expect(formatTimestamp('2026-06-12T14:26:40Z')).not.toBe('N/A');
    expect(formatTimestamp(1_700_000_000)).not.toBe('N/A');
  });
});

describe('timestampAgeSeconds', () => {
  it('is ~0 for now and Infinity for garbage', () => {
    expect(timestampAgeSeconds(new Date().toISOString())).toBeLessThan(2);
    expect(timestampAgeSeconds('nonsense')).toBe(Number.POSITIVE_INFINITY);
  });
});

describe('getErrorMessage', () => {
  it('unwraps Errors, strings, and falls back', () => {
    expect(getErrorMessage(new Error('boom'))).toBe('boom');
    expect(getErrorMessage('plain')).toBe('plain');
    expect(getErrorMessage(null, 'fallback')).toBe('fallback');
  });
});

describe('clamp / radiansToDegrees', () => {
  it('clamps to range', () => {
    expect(clamp(5, 0, 10)).toBe(5);
    expect(clamp(-1, 0, 10)).toBe(0);
    expect(clamp(99, 0, 10)).toBe(10);
  });
  it('converts radians to degrees', () => {
    expect(radiansToDegrees(Math.PI)).toBeCloseTo(180);
  });
});
