import { useEffect, useRef } from 'react'
import { useAppState } from '../state/appState'

export const useCanvasRender = () => {
  const canvasRef = useRef<HTMLCanvasElement | null>(null)
  const { zoom, selectedGalleryItem } = useAppState()

  useEffect(() => {
    if (!canvasRef.current) return

    const ctx = canvasRef.current.getContext('2d')
    if (!ctx) return

    ctx.clearRect(0, 0, canvasRef.current.width, canvasRef.current.height)
    ctx.fillStyle = '#272639'
    ctx.fillRect(0, 0, canvasRef.current.width, canvasRef.current.height)

    if (selectedGalleryItem) {
      const image = new Image()
      image.src = selectedGalleryItem.src
      image.onload = () => {
        const scale = Math.min(
          (canvasRef.current!.width - 80) / image.width,
          (canvasRef.current!.height - 80) / image.height,
        )
        const drawWidth = image.width * scale * zoom
        const drawHeight = image.height * scale * zoom
        ctx.drawImage(
          image,
          (canvasRef.current!.width - drawWidth) / 2,
          (canvasRef.current!.height - drawHeight) / 2,
          drawWidth,
          drawHeight,
        )
      }
    }
  }, [selectedGalleryItem, zoom])

  return canvasRef
}
