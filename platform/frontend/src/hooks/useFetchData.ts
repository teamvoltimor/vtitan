/**
 * src/hooks/useFetchData.ts
 * 
 * Reusable hook for async data fetching with unified error handling.
 * Eliminates duplicate try-catch and loading state logic throughout app.
 */

import { useEffect, useRef, useState } from 'react'
import { getErrorMessage } from '../utils/formatting'

/**
 * Options for useFetchData hook.
 */
export interface UseFetchDataOptions<T> {
  /**
   * Manual trigger function to re-fetch data.
   * Separate from dependency-based automatic fetching.
   */
  onRetry?: () => void

  /**
   * Optional callback when fetch completes successfully.
   */
  onSuccess?: (data: T) => void

  /**
   * Optional callback when fetch encounters an error.
   */
  onError?: (error: string) => void

  /**
   * Skip this fetch entirely (useful for conditional fetching).
   */
  skip?: boolean
}

/**
 * Return value from useFetchData hook.
 */
export interface UseFetchDataResult<T> {
  data: T | null
  loading: boolean
  error: string | null
  retry: () => void
}

/**
 * Reusable hook for async data fetching with error handling.
 * 
 * Handles:
 * - Async data fetching with loading state
 * - Error handling with user-friendly messages
 * - Memory leak prevention (mounted flag)
 * - Manual retry capability
 * - Dependency-based re-fetching
 * 
 * @example
 * const { data, loading, error, retry } = useFetchData(
 *   () => fetchTelemetryData(),
 *   [retryCount]
 * )
 * 
 * if (loading) return <LoadingSpinner />
 * if (error) return <ErrorMessage error={error} onRetry={retry} />
 * return <DataDisplay data={data} />
 */
export function useFetchData<T>(
  fetchFn: () => Promise<T>,
  dependencies: React.DependencyList = [],
  options: UseFetchDataOptions<T> = {}
): UseFetchDataResult<T> {
  const { onRetry, onSuccess, onError, skip = false } = options

  const [data, setData] = useState<T | null>(null)
  const [loading, setLoading] = useState(true)
  const [error, setError] = useState<string | null>(null)
  const [retryCount, setRetryCount] = useState(0)

  // Track if component is mounted to prevent state updates after unmount
  const mounted = useRef(true)

  // Cleanup on unmount
  useEffect(() => {
    return () => {
      mounted.current = false
    }
  }, [])

  // Main fetch effect
  useEffect(() => {
    // Guard clause: skip this fetch
    if (skip) {
      setLoading(false)
      return
    }

    let isMounted = true

    const execute = async () => {
      setLoading(true)
      setError(null)

      try {
        const result = await fetchFn()

        // Guard clause: component unmounted or effect was cancelled
        if (!isMounted || !mounted.current) {
          return
        }

        setData(result)
        setError(null)
        onSuccess?.(result)
      } catch (err) {
        // Guard clause: component unmounted or effect was cancelled
        if (!isMounted || !mounted.current) {
          return
        }

        const message = getErrorMessage(err, 'Failed to load data')
        setError(message)
        onError?.(message)
      } finally {
        if (isMounted) {
          setLoading(false)
        }
      }
    }

    execute()

    // Cleanup function for this effect
    return () => {
      isMounted = false
    }
  }, [fetchFn, retryCount, ...dependencies, skip, onSuccess, onError])

  // Manual retry handler
  const retry = () => {
    onRetry?.()
    setRetryCount((c) => c + 1)
  }

  return { data, loading, error, retry }
}

/**
 * Hook for multiple concurrent fetches with unified error handling.
 * 
 * @example
 * const { data, loading, errors, retry } = useFetchDataConcurrent([
 *   () => fetchUsers(),
 *   () => fetchPosts(),
 *   () => fetchComments(),
 * ])
 */
export function useFetchDataConcurrent<T extends readonly unknown[]>(
  fetchFns: { readonly [K in keyof T]: () => Promise<T[K]> },
  dependencies: React.DependencyList = []
): {
  data: T | null
  loading: boolean
  errors: (string | null)[]
  retry: () => void
} {
  const [data, setData] = useState<T | null>(null)
  const [loading, setLoading] = useState(true)
  const [errors, setErrors] = useState<(string | null)[]>(
    Array(fetchFns.length).fill(null)
  )
  const [retryCount, setRetryCount] = useState(0)
  const mounted = useRef(true)

  useEffect(() => {
    return () => {
      mounted.current = false
    }
  }, [])

  useEffect(() => {
    let isMounted = true

    const execute = async () => {
      setLoading(true)
      setErrors(Array(fetchFns.length).fill(null))

      try {
        const results = await Promise.all(
          fetchFns.map((fn) => fn().catch((err) => err))
        )

        // Guard clause: component unmounted
        if (!isMounted || !mounted.current) return

        // Separate successful results from errors
        const newErrors: (string | null)[] = []
        const successResults: any[] = []

        for (let i = 0; i < results.length; i++) {
          if (results[i] instanceof Error) {
            newErrors.push(getErrorMessage(results[i]))
            successResults.push(null)
          } else {
            newErrors.push(null)
            successResults.push(results[i])
          }
        }

        setData(successResults as unknown as T)
        setErrors(newErrors)
      } catch (err) {
        // Guard clause: component unmounted
        if (!isMounted || !mounted.current) return

        const message = getErrorMessage(err, 'Failed to load data')
        setErrors(Array(fetchFns.length).fill(message))
      } finally {
        if (isMounted) {
          setLoading(false)
        }
      }
    }

    execute()

    return () => {
      isMounted = false
    }
  }, [fetchFns.length, retryCount, ...dependencies])

  const retry = () => {
    setRetryCount((c) => c + 1)
  }

  return { data, loading, errors, retry }
}
