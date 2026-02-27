import { createContext, useCallback, useContext, useMemo, useState } from 'react'

export type PointType = 'positive' | 'negative'
export type ExportFormat = 'segmentation' | 'detection'
export type ViewMode = 'List' | 'Grid'
export type OutlineMode = 'Class color' | 'Neutral'

export interface GalleryItem {
  id: string
  label: string
  src: string
  format: string
  status: 'pending' | 'done' | 'skipped'
  updated: string
}

export interface TimelineEvent {
  time: string
  label: string
}

export interface ModelOption {
  id: string
  label: string
}

interface AppStateContextValue {
  zoom: number
  setZoom: (value: number) => void
  pointType: PointType
  setPointType: (value: PointType) => void
  exportFormat: ExportFormat
  setExportFormat: (value: ExportFormat) => void
  maskLevel: string
  setMaskLevel: (value: string) => void
  activeClass: string | null
  setActiveClass: (value: string | null) => void
  classes: string[]
  classColors: Record<string, string>
  addClass: (name: string) => void
  updateClassColor: (name: string, color: string) => void
  logEntries: string[]
  pushLog: (entry: string) => void
  stats: { processed: string; skipped: string; labels: string }
  setStats: (stats: AppStateContextValue['stats']) => void
  gallery: GalleryItem[]
  refreshGallery: () => void
  importImages: (files: FileList | null) => void
  selectedGalleryItem: GalleryItem | null
  setSelectedGalleryItem: (item: GalleryItem | null) => void
  viewMode: ViewMode
  setViewMode: (mode: ViewMode) => void
  outlineMode: OutlineMode
  setOutlineMode: (mode: OutlineMode) => void
  timeline: TimelineEvent[]
  pushTimeline: (event: TimelineEvent) => void
  recordAction: (label: string) => void
  models: ModelOption[]
  selectedModel: string
  modelStatus: string
  loadModel: (modelId: string) => void
  autoAnnotate: () => void
}

const AppStateContext = createContext<AppStateContextValue | null>(null)

const modelOptions: ModelOption[] = [
  { id: 'sam_vit_b', label: 'SAM ViT-B' },
  { id: 'sam_vit_l', label: 'SAM ViT-L' },
  { id: 'sam_vit_h', label: 'SAM ViT-H' },
]

const galleryStatus: Array<'pending' | 'done' | 'skipped'> = ['done', 'pending', 'skipped']
const makeGallery = (srcSeed: string): GalleryItem[] =>
  Array.from({ length: 6 }).map((_, index) => ({
    id: `img-${index}`,
    label: `Sample ${index + 1}`,
    src: `https://picsum.photos/seed/${srcSeed}-${index}/400/280`,
    format: index % 2 === 0 ? 'png' : 'jpeg',
    status: galleryStatus[index % galleryStatus.length],
    updated: `${index + 2}m ago`,
  }))

export const AppProvider = ({ children }: { children: React.ReactNode }) => {
  const [zoom, setZoom] = useState(1)
  const [pointType, setPointType] = useState<PointType>('positive')
  const [exportFormat, setExportFormat] = useState<ExportFormat>('segmentation')
  const [maskLevel, setMaskLevel] = useState('Object (1)')
  const [activeClass, setActiveClass] = useState<string | null>(null)
  const [classes, setClasses] = useState(['Foreground', 'Background'])
  const [classColors, setClassColors] = useState<Record<string, string>>({
    Foreground: '#89b4fa',
    Background: '#cba6f7',
  })
  const [logEntries, setLogEntries] = useState<string[]>(['Ready.'])
  const [stats, setStats] = useState({ processed: '12', skipped: '1', labels: '4' })
  const [gallery, setGallery] = useState<GalleryItem[]>(makeGallery('auto'))
  const [selectedGalleryItem, setSelectedGalleryItem] = useState<GalleryItem | null>(gallery[0] ?? null)
  const [viewMode, setViewMode] = useState<ViewMode>('List')
  const [outlineMode, setOutlineMode] = useState<OutlineMode>('Class color')
  const [timeline, setTimeline] = useState<TimelineEvent[]>([
    { time: '09:18', label: 'Gallery seeded' },
    { time: '09:20', label: 'Loaded demo set' },
  ])
  const [selectedModel, setSelectedModel] = useState(modelOptions[0].id)
  const [modelStatus, setModelStatus] = useState('No model loaded')

  const pushLog = useCallback((entry: string) => {
    setLogEntries((prev) => [entry, ...prev].slice(0, 5))
  }, [])

  const pushTimeline = useCallback((event: TimelineEvent) => {
    setTimeline((prev) => [event, ...prev].slice(0, 6))
  }, [])

  const recordAction = useCallback(
    (label: string) => {
      const time = new Date().toLocaleTimeString([], { hour: '2-digit', minute: '2-digit' })
      pushLog(`${time} · ${label}`)
      pushTimeline({ time, label })
    },
    [pushLog, pushTimeline],
  )

  const addClass = useCallback(
    (name: string) => {
      setClasses((prev) => (prev.includes(name) ? prev : [...prev, name]))
      setActiveClass(name)
      setClassColors((prev) => ({ ...prev, [name]: '#fe9664' }))
      recordAction(`Created class ${name}`)
    },
    [recordAction],
  )

  const updateClassColor = useCallback(
    (name: string, color: string) => {
      setClassColors((prev) => ({ ...prev, [name]: color }))
      recordAction(`Updated color for ${name}`)
    },
    [recordAction],
  )

  const refreshGallery = useCallback(() => {
    setGallery(makeGallery('refresh'))
    recordAction('Gallery refreshed')
    setStats((prev) => ({ ...prev, processed: (parseInt(prev.processed, 10) + 3).toString() }))
  }, [recordAction])

  const importImages = useCallback(
    (files: FileList | null) => {
      if (!files || files.length === 0) {
        return
      }
      const imported: GalleryItem[] = Array.from(files).map((file, index) => ({
        id: `import-${Date.now()}-${index}`,
        label: file.name,
        src: `https://picsum.photos/seed/import-${Date.now()}-${index}/400/280`,
        format: file.type.split('/')[1] || 'jpg',
        status: 'pending',
        updated: 'just now',
      }))
      setGallery((prev) => [...imported, ...prev])
      recordAction(`Imported ${files.length} image${files.length > 1 ? 's' : ''}`)
      setStats((prev) => ({
        ...prev,
        processed: (parseInt(prev.processed, 10) + files.length).toString(),
        labels: (parseInt(prev.labels, 10) + files.length).toString(),
      }))
    },
    [recordAction],
  )

  const loadModel = useCallback(
    (modelId: string) => {
      setSelectedModel(modelId)
      setModelStatus(`Loaded ${modelId}`)
      recordAction(`Model ${modelId} loaded`)
    },
    [recordAction],
  )

  const autoAnnotate = useCallback(() => {
    recordAction('Auto-annotated current image')
    setStats((prev) => ({ ...prev, labels: (parseInt(prev.labels, 10) + 2).toString() }))
  }, [recordAction])

  const value = useMemo(
    () => ({
      zoom,
      setZoom,
      pointType,
      setPointType,
      exportFormat,
      setExportFormat,
      maskLevel,
      setMaskLevel,
      activeClass,
      setActiveClass,
      classes,
      classColors,
      addClass,
      updateClassColor,
      logEntries,
      pushLog,
      stats,
      setStats,
      gallery,
      refreshGallery,
      importImages,
      selectedGalleryItem,
      setSelectedGalleryItem,
      viewMode,
      setViewMode,
      outlineMode,
      setOutlineMode,
      timeline,
      pushTimeline,
      recordAction,
      models: modelOptions,
      selectedModel,
      modelStatus,
      loadModel,
      autoAnnotate,
    }),
    [
      zoom,
      pointType,
      exportFormat,
      maskLevel,
      activeClass,
      classes,
      classColors,
      logEntries,
      stats,
      gallery,
      selectedGalleryItem,
      viewMode,
      outlineMode,
      timeline,
      selectedModel,
      modelStatus,
      addClass,
      updateClassColor,
      pushLog,
      refreshGallery,
      importImages,
      setSelectedGalleryItem,
      setViewMode,
      setOutlineMode,
      pushTimeline,
      recordAction,
      loadModel,
      autoAnnotate,
    ],
  )

  return <AppStateContext.Provider value={value}>{children}</AppStateContext.Provider>
}

export const useAppState = () => {
  const ctx = useContext(AppStateContext)
  if (!ctx) {
    throw new Error('useAppState must be used within AppProvider')
  }
  return ctx
}
