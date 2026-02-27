import { useCallback, useEffect, useState } from 'react'
import type { GalleryItem } from '../state/appState'

const fetchGallery = (): Promise<GalleryItem[]> =>
  new Promise((resolve) => {
    setTimeout(() => {
      const items = Array.from({ length: 9 }).map((_, index) => ({
        id: `demo-${index}`,
        label: `Demo ${index + 1}`,
        src: `https://picsum.photos/seed/auto-${index}/450/300`,
        format: 'png',
        status: 'pending' as const,
        updated: 'just now',
      }))
      resolve(items)
    }, 320)
  })

export const useGallery = () => {
  const [items, setItems] = useState<GalleryItem[]>([])
  const [loading, setLoading] = useState(true)

  const load = useCallback(async () => {
    setLoading(true)
    const gallery = await fetchGallery()
    setItems(gallery)
    setLoading(false)
  }, [])

  useEffect(() => {
    load()
  }, [load])

  return { items, loading, refresh: load }
}
