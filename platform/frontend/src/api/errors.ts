/**
 * src/api/errors.ts
 * 
 * Telemetry-specific error types with classification for retry logic.
 * Follows SOLID principles: error handling is centralized and reusable.
 */

/**
 * Typed error for telemetry operations.
 * Distinguishes between retryable (network) and permanent (validation) errors.
 */
export class TelemetryError extends Error {
  readonly code:
    | 'NETWORK'
    | 'PARSE'
    | 'VALIDATION'
    | 'TIMEOUT'
    | 'UNAUTHORIZED'
    | 'NOT_FOUND'
    | 'SERVER_ERROR'

  readonly statusCode?: number
  readonly originalError?: unknown

  constructor(
    code:
      | 'NETWORK'
      | 'PARSE'
      | 'VALIDATION'
      | 'TIMEOUT'
      | 'UNAUTHORIZED'
      | 'NOT_FOUND'
      | 'SERVER_ERROR',
    message: string,
    statusCode?: number,
    originalError?: unknown
  ) {
    super(message)
    this.name = 'TelemetryError'
    this.code = code
    this.statusCode = statusCode
    this.originalError = originalError
    Object.setPrototypeOf(this, TelemetryError.prototype)
  }

  /**
   * Determine if this error should trigger a retry.
   * Retryable: network issues, timeouts, server errors
   * Not retryable: validation errors, auth errors, 404s
   */
  isRetryable(): boolean {
    return (
      this.code === 'NETWORK' ||
      this.code === 'TIMEOUT' ||
      this.code === 'SERVER_ERROR'
    )
  }

  /**
   * Determine if this error is permanent and won't be resolved by retrying.
   */
  isPermanent(): boolean {
    return (
      this.code === 'VALIDATION' ||
      this.code === 'UNAUTHORIZED' ||
      this.code === 'NOT_FOUND'
    )
  }
}

/**
 * Exponential backoff calculator for retry logic.
 * Prevents overwhelming servers during outages.
 * 
 * @example
 * const backoff = new ExponentialBackoff(1000, 30000, 5, 1.5)
 * const { delay, canRetry } = backoff.getNextDelay()
 * // delay = 1000ms, canRetry = true
 * // Next call: delay = 1500ms, canRetry = true
 * // After 5 attempts: canRetry = false
 */
export class ExponentialBackoff {
  private attemptCount = 0

  private readonly initialDelayMs: number
  private readonly maxDelayMs: number
  private readonly maxAttempts: number
  private readonly multiplier: number

  constructor(
    initialDelayMs: number = 1000,
    maxDelayMs: number = 30000,
    maxAttempts: number = 5,
    multiplier: number = 1.5
  ) {
    this.initialDelayMs = initialDelayMs
    this.maxDelayMs = maxDelayMs
    this.maxAttempts = maxAttempts
    this.multiplier = multiplier
  }

  /**
   * Calculate the next delay before retrying.
   * Returns delay in milliseconds and whether more retries are possible.
   */
  getNextDelay(): { delay: number; canRetry: boolean } {
    // Guard clause: max attempts reached
    if (this.attemptCount >= this.maxAttempts) {
      return { delay: 0, canRetry: false }
    }

    // Calculate exponential delay with cap
    const delay = Math.min(
      this.initialDelayMs * Math.pow(this.multiplier, this.attemptCount),
      this.maxDelayMs
    )

    this.attemptCount++

    return { delay, canRetry: this.attemptCount < this.maxAttempts }
  }

  /**
   * Reset attempt counter (e.g., after successful connection).
   */
  reset(): void {
    this.attemptCount = 0
  }

  /**
   * Get current attempt number.
   */
  getAttemptCount(): number {
    return this.attemptCount
  }
}

/**
 * Helper to classify fetch/HTTP errors.
 */
export function classifyHttpError(statusCode: number): TelemetryError['code'] {
  if (statusCode === 401) return 'UNAUTHORIZED'
  if (statusCode === 404) return 'NOT_FOUND'
  if (statusCode >= 500) return 'SERVER_ERROR'
  if (statusCode >= 400) return 'VALIDATION'
  return 'NETWORK'
}
