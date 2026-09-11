/**
 * src/config/colors.ts
 *
 * Colour palette. Used by JS/TSX-rendered colours: three.js materials, canvas
 * drawing, and SVG. Where a colour also exists in CSS it must match the
 * custom property in index.css (--color-*) — they are the shared source of
 * truth and must not drift (audit §8.4). Note SURFACE is the *opaque* solid
 * used for the 3D floor; the stylesheet's --color-surface is a translucent
 * overlay and intentionally different.
 */

export const COLORS = {
  BACKGROUND: '#050b12',
  PANEL: '#0a1525',
  SURFACE: '#111b27',
  ACCENT: '#ff8a65',
  HIGHLIGHT: '#5fdde5',
  WHITE: '#ffffff',
  BLUE: '#2196f3',
  DANGER: '#f44336',
  WARNING: '#ff9800',
  SUCCESS: '#4caf50',
  ERROR: '#ff0000',
  GRID_LINE: 'rgba(255, 255, 255, 0.1)',
  FORWARD_INDICATOR: 'rgba(255, 255, 255, 0.3)',
} as const;

// THEME COLORS

export const THEME = {
  COLORS: {
    BACKGROUND: COLORS.BACKGROUND,
    PANEL: COLORS.PANEL,
    ACCENT: COLORS.ACCENT,
    HIGHLIGHT: COLORS.HIGHLIGHT,
    ERROR: COLORS.ERROR,
    SUCCESS: COLORS.SUCCESS,
    WARNING: COLORS.WARNING,
  },
} as const;
