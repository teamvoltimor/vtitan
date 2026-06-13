/**
 * src/utils/formatting.ts
 *
 * Shared formatting helpers. Guard clauses first, early returns.
 */

/**
 * Format a number with optional unit and decimal places.
 *
 * @example
 * formatNumber(1.2345) // "1.23"
 * formatNumber(1.2345, { unit: 'm/s' }) // "1.23 m/s"
 * formatNumber(null, { default: 'N/A' }) // "N/A"
 */
export function formatNumber(
  value: number | null | undefined,
  options: { decimals?: number; unit?: string; default?: string } = {}
): string {
  const { decimals = 2, unit = '', default: fallback = 'N/A' } = options

  if (value == null) {
    return fallback
  }

  const formatted = value.toFixed(decimals)
  return unit ? `${formatted} ${unit}` : formatted
}

/**
 * Format a timestamp to readable time or full date. Accepts an ISO-8601 string
 * (as emitted by the Go backend via protojson) or a Unix-seconds number.
 *
 * @example
 * formatTimestamp('2026-06-12T14:26:40Z')         // "14:26:40"
 * formatTimestamp('2026-06-12T14:26:40Z', 'full') // "6/12/2026, 2:26:40 PM"
 */
export function formatTimestamp(
  value: string | number | null | undefined,
  format: 'time' | 'full' = 'time'
): string {
  if (value == null) {
    return 'N/A'
  }

  const date = typeof value === 'number' ? new Date(value * 1000) : new Date(value)
  if (Number.isNaN(date.getTime())) {
    return 'N/A'
  }

  return format === 'time' ? date.toLocaleTimeString() : date.toLocaleString()
}

/** Age in seconds of an ISO-8601 timestamp relative to now. */
export function timestampAgeSeconds(iso: string): number {
  const ms = new Date(iso).getTime()
  return Number.isNaN(ms) ? Number.POSITIVE_INFINITY : (Date.now() - ms) / 1000
}

/**
 * Extract a message from an unknown error value.
 *
 * @example
 * getErrorMessage(new Error('Failed')) // "Failed"
 * getErrorMessage(null, 'Custom fallback') // "Custom fallback"
 */
export function getErrorMessage(error: unknown, fallback: string = 'An error occurred'): string {
  if (error instanceof Error) {
    return error.message
  }
  if (typeof error === 'string') {
    return error
  }
  return fallback
}

/** Clamp a number to the [min, max] range. */
export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value))
}

/** Convert radians to degrees. */
export function radiansToDegrees(radians: number): number {
  return radians * (180 / Math.PI)
}
