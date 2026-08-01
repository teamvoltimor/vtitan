/**
 * src/config/index.ts
 *
 * Barrel — re-export every configuration module from a single entry point so
 * consumers keep a single `from '../config'` import (audit §4.4).
 */

export * from './api.config';
export * from './colors';
export * from './scene.config';
export * from './ui.config';
