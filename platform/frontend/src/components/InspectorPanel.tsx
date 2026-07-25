import { useState, useRef, useEffect } from 'react';
import type { TopicsSnapshot } from '../types';
import { TopicInspector } from './TopicInspector';

const INSPECTOR_MIN_HEIGHT = 150;
const INSPECTOR_MAX_RATIO = 0.85;

export function InspectorPanel({ topics }: { topics: TopicsSnapshot | null }) {
  const [inspectorHeight, setInspectorHeight] = useState(300);
  const isResizing = useRef(false);
  const startY = useRef(0);
  const startHeight = useRef(0);

  const onResizeStart = (e: React.MouseEvent) => {
    isResizing.current = true;
    startY.current = e.clientY;
    startHeight.current = inspectorHeight;
    e.preventDefault();
  };

  const onResizeKeyDown = (e: React.KeyboardEvent) => {
    const step = 20;
    const max = window.innerHeight * INSPECTOR_MAX_RATIO;
    if (e.key === 'ArrowUp') {
      setInspectorHeight((h) => Math.min(max, h + step));
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      setInspectorHeight((h) => Math.max(INSPECTOR_MIN_HEIGHT, h - step));
      e.preventDefault();
    }
  };

  useEffect(() => {
    const onMove = (e: MouseEvent) => {
      if (!isResizing.current) return;
      const delta = startY.current - e.clientY;
      const max = window.innerHeight * INSPECTOR_MAX_RATIO;
      setInspectorHeight(
        Math.max(INSPECTOR_MIN_HEIGHT, Math.min(max, startHeight.current + delta))
      );
    };
    const onUp = () => {
      isResizing.current = false;
    };
    document.addEventListener('mousemove', onMove);
    document.addEventListener('mouseup', onUp);
    return () => {
      document.removeEventListener('mousemove', onMove);
      document.removeEventListener('mouseup', onUp);
    };
  }, []);

  return (
    <div className="inspector-panel" style={{ height: inspectorHeight }}>
      <div
        className="inspector-resize-handle"
        onMouseDown={onResizeStart}
        onKeyDown={onResizeKeyDown}
        aria-label="Drag to resize"
        role="separator"
        aria-orientation="horizontal"
        aria-valuenow={inspectorHeight}
        aria-valuemin={INSPECTOR_MIN_HEIGHT}
        aria-valuemax={Math.round(window.innerHeight * INSPECTOR_MAX_RATIO)}
        tabIndex={0}
      />
      <TopicInspector topics={topics} />
    </div>
  );
}
