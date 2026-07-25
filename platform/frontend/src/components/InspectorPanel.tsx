import { useState, useRef, useEffect } from 'react';
import type { TopicsSnapshot } from '../types';
import { TopicInspector } from './TopicInspector';

const INSPECTOR_MIN_HEIGHT = 150;
const INSPECTOR_MAX_RATIO = 0.85;

export function InspectorPanel({ topics }: { topics: TopicsSnapshot | null }) {
  const [inspectorHeight, setInspectorHeight] = useState(300);
  const [viewportHeight, setViewportHeight] = useState(() => window.innerHeight);
  const panelRef = useRef<HTMLDivElement>(null);
  const isResizing = useRef(false);
  const startY = useRef(0);
  const startHeight = useRef(0);

  const maxHeight = viewportHeight * INSPECTOR_MAX_RATIO;

  useEffect(() => {
    const onResize = () => setViewportHeight(window.innerHeight);
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, []);

  const onResizeStart = (e: React.MouseEvent) => {
    isResizing.current = true;
    startY.current = e.clientY;
    startHeight.current = inspectorHeight;
    document.body.style.userSelect = 'none';
    e.preventDefault();
  };

  const onResizeKeyDown = (e: React.KeyboardEvent) => {
    const step = 20;
    if (e.key === 'ArrowUp') {
      setInspectorHeight((h) => Math.min(maxHeight, h + step));
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      setInspectorHeight((h) => Math.max(INSPECTOR_MIN_HEIGHT, h - step));
      e.preventDefault();
    }
  };

  useEffect(() => {
    // Write directly to the element during drag instead of calling
    // setState per mousemove — a state update per pixel dragged re-renders
    // the whole topic list (and any expanded visualizer) on every tick.
    // The height is committed to state once, on mouseup.
    const onMove = (e: MouseEvent) => {
      if (!isResizing.current || !panelRef.current) return;
      const delta = startY.current - e.clientY;
      const next = Math.max(INSPECTOR_MIN_HEIGHT, Math.min(maxHeight, startHeight.current + delta));
      panelRef.current.style.height = `${next}px`;
    };
    const onUp = () => {
      if (!isResizing.current) return;
      isResizing.current = false;
      document.body.style.userSelect = '';
      if (panelRef.current) {
        setInspectorHeight(Number.parseFloat(panelRef.current.style.height) || inspectorHeight);
      }
    };
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
    return () => {
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    };
  }, [maxHeight, inspectorHeight]);

  return (
    <div ref={panelRef} className="inspector-panel" style={{ height: inspectorHeight }}>
      {/* biome-ignore lint/a11y/useSemanticElements: this is the WAI-ARIA APG "movable separator" pattern (focusable, aria-valuenow) — <hr> can't be interactive */}
      <div
        className="inspector-resize-handle"
        onMouseDown={onResizeStart}
        onKeyDown={onResizeKeyDown}
        aria-label="Drag to resize"
        role="separator"
        aria-orientation="horizontal"
        aria-valuenow={Math.round(inspectorHeight)}
        aria-valuemin={INSPECTOR_MIN_HEIGHT}
        aria-valuemax={Math.round(maxHeight)}
        tabIndex={0}
      />
      <TopicInspector topics={topics} />
    </div>
  );
}
