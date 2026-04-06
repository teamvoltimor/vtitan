/**
 * src/utils/formatting.ts
 * 
 * Shared formatting utilities to eliminate code duplication.
 * Follows the logic-cleaner principle: guard clauses first, early returns.
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
  options: {
    decimals?: number
    unit?: string
    default?: string
  } = {}
): string {
  const { decimals = 2, unit = '', default: fallback = 'N/A' } = options

  // Guard clause: null/undefined check
  if (value == null) {
    return fallback
  }

  const formatted = value.toFixed(decimals)
  return unit ? `${formatted} ${unit}` : formatted
}

/**
 * Format a Unix timestamp (seconds) to readable time or full date.
 * 
 * @example
 * formatUnixTimestamp(1680000000) // "14:26:40"
 * formatUnixTimestamp(1680000000, 'full') // "3/28/2023, 2:26:40 PM"
 * formatUnixTimestamp(null) // "N/A"
 */
export function formatUnixTimestamp(
  unixSeconds: number | null | undefined,
  format: 'time' | 'full' = 'time'
): string {
  // Guard clause: null/undefined check
  if (unixSeconds == null) {
    return 'N/A'
  }

  const date = new Date(unixSeconds * 1000)

  return format === 'time' ? date.toLocaleTimeString() : date.toLocaleString()
}

/**
 * Extract error message from unknown error type.
 * Handles Error objects, strings, and unknown types gracefully.
 * 
 * @example
 * getErrorMessage(new Error('Failed')) // "Failed"
 * getErrorMessage('timeout') // "timeout"
 * getErrorMessage(null) // "An error occurred"
 * getErrorMessage(null, 'Custom fallback') // "Custom fallback"
 */
export function getErrorMessage(
  error: unknown,
  fallback: string = 'An error occurred'
): string {
  // Guard clause: Error instance
  if (error instanceof Error) {
    return error.message
  }

  // Guard clause: string type
  if (typeof error === 'string') {
    return error
  }

  // Default fallback
  return fallback
}

/**
 * Safe deep property access with default value.
 * Handles null/undefined traversal safely.
 * 
 * @example
 * safeGet({ a: { b: 1 } }, 'a.b') // 1
 * safeGet({ a: null }, 'a.b', 99) // 99
 * safeGet(null, 'a.b', 99) // 99
 */
export function safeGet<T>(
  obj: unknown,
  path: string,
  defaultValue: T
): T {
  const keys = path.split('.')
  let current: any = obj

  for (const key of keys) {
    // Guard clause: null/undefined check
    if (current == null) {
      return defaultValue
    }

    current = current[key]
  }

  return current ?? defaultValue
}

/**
 * Format array of values as readable list.
 * 
 * @example
 * formatList([1, 2, 3]) // "1, 2, 3"
 * formatList([1, 2, 3], 'and') // "1, 2, and 3"
 */
export function formatList(
  items: (string | number)[],
  conjunction: 'and' | 'or' = 'and'
): string {
  // Guard clause: empty array
  if (items.length === 0) {
    return ''
  }

  // Guard clause: single item
  if (items.length === 1) {
    return String(items[0])
  }

  // Guard clause: two items
  if (items.length === 2) {
    return `${items[0]} ${conjunction} ${items[1]}`
  }

  // Multiple items: use oxford comma
  const lastItem = items[items.length - 1]
  const otherItems = items.slice(0, -1)
  return `${otherItems.join(', ')}, ${conjunction} ${lastItem}`
}

/**
 * Clamp a number between min and max.
 * 
 * @example
 * clamp(5, 0, 10) // 5
 * clamp(-5, 0, 10) // 0
 * clamp(15, 0, 10) // 10
 */
export function clamp(value: number, min: number, max: number): number {
  return Math.max(min, Math.min(max, value))
}

/**
 * Convert angle from radians to degrees.
 */
export function radiansToDegrees(radians: number): number {
  return radians * (180 / Math.PI)
}

/**
 * Convert angle from degrees to radians.
 */
export function degreesToRadians(degrees: number): number {
  return degrees * (Math.PI / 180)
}

/**
 * Normalize angle to [0, 2π) range.
 */
export function normalizeAngle(angle: number): number {
  let normalized = angle % (Math.PI * 2)
  if (normalized < 0) {
    normalized += Math.PI * 2
  }
  return normalized
}

/**
 * Normalize angle to [-π, π] range (centered at 0).
 */
export function normalizeAngleSigned(angle: number): number {
  let normalized = normalizeAngle(angle)
  if (normalized > Math.PI) {
    normalized -= Math.PI * 2
  }
  return normalized
}
