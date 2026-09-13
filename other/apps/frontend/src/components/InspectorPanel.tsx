import { useEffect, useRef, useState } from 'react';
import type { TopicsSnapshot } from '../types';
import { TopicInspector } from './TopicInspector';

const INSPECTOR_MIN_HEIGHT = 150;
const INSPECTOR_MAX_RATIO = 0.85;
const KEYBOARD_STEP = 20;
const KEYBOARD_COARSE_STEP = 80;

export function InspectorPanel({ topics }: { topics: TopicsSnapshot | null }) {
  const [inspectorHeight, setInspectorHeight] = useState(() => {
    const fromRoot = Number.parseFloat(
      getComputedStyle(document.documentElement).getPropertyValue('--inspector-height')
    );
    return Number.isNaN(fromRoot) ? 300 : fromRoot;
  });
  const [viewportHeight, setViewportHeight] = useState(() => window.innerHeight);
  const panelRef = useRef<HTMLDivElement>(null);
  const startY = useRef(0);
  const startHeight = useRef(0);

  const maxHeight = viewportHeight * INSPECTOR_MAX_RATIO;

  const setInspectorHeightVar = (next: number) => {
    const shell = panelRef.current?.parentElement;
    if (shell instanceof HTMLElement) {
      shell.style.setProperty('--inspector-height', `${next}px`);
    }
  };

  useEffect(() => {
    const onResize = () => setViewportHeight(window.innerHeight);
    window.addEventListener('resize', onResize);
    return () => window.removeEventListener('resize', onResize);
  }, []);

  const onResizeStart = (e: React.PointerEvent<HTMLDivElement>) => {
    startY.current = e.clientY;
    startHeight.current = inspectorHeight;
    e.currentTarget.setPointerCapture(e.pointerId);
    e.preventDefault();
  };

  const onResizeMove = (e: React.PointerEvent<HTMLDivElement>) => {
    if (!e.currentTarget.hasPointerCapture(e.pointerId)) return;
    const delta = startY.current - e.clientY;
    // Write straight to the CSS variable during the drag instead of setState
    // per move — a state update per pixel re-renders the whole topic list.
    const next = Math.max(INSPECTOR_MIN_HEIGHT, Math.min(maxHeight, startHeight.current + delta));
    setInspectorHeightVar(next);
  };

  const onResizeEnd = (e: React.PointerEvent<HTMLDivElement>) => {
    if (!e.currentTarget.hasPointerCapture(e.pointerId)) return;
    e.currentTarget.releasePointerCapture(e.pointerId);
    const shell = panelRef.current?.parentElement;
    const current =
      shell instanceof HTMLElement
        ? Number.parseFloat(shell.style.getPropertyValue('--inspector-height'))
        : Number.NaN;
    setInspectorHeight(Number.isNaN(current) ? inspectorHeight : current);
  };

  const commitHeight = (next: number) => {
    const clamped = Math.max(INSPECTOR_MIN_HEIGHT, Math.min(maxHeight, next));
    setInspectorHeightVar(clamped);
    setInspectorHeight(clamped);
  };

  const onResizeKeyDown = (e: React.KeyboardEvent) => {
    const step = e.shiftKey ? KEYBOARD_COARSE_STEP : KEYBOARD_STEP;
    if (e.key === 'ArrowUp') {
      commitHeight(inspectorHeight + step);
      e.preventDefault();
    } else if (e.key === 'ArrowDown') {
      commitHeight(inspectorHeight - step);
      e.preventDefault();
    } else if (e.key === 'PageUp') {
      commitHeight(inspectorHeight + KEYBOARD_COARSE_STEP);
      e.preventDefault();
    } else if (e.key === 'PageDown') {
      commitHeight(inspectorHeight - KEYBOARD_COARSE_STEP);
      e.preventDefault();
    } else if (e.key === 'Home') {
      commitHeight(INSPECTOR_MIN_HEIGHT);
      e.preventDefault();
    } else if (e.key === 'End') {
      commitHeight(maxHeight);
      e.preventDefault();
    }
  };

  return (
    <div ref={panelRef} id="topic-inspector" className="inspector-panel">
      {/* biome-ignore lint/a11y/useSemanticElements: this is the WAI-ARIA APG "movable separator" pattern (focusable, aria-valuenow) — <hr> can't be interactive */}
      <div
        className="inspector-resize-handle"
        onPointerDown={onResizeStart}
        onPointerMove={onResizeMove}
        onPointerUp={onResizeEnd}
        onPointerCancel={onResizeEnd}
        onKeyDown={onResizeKeyDown}
        aria-label="Drag to resize"
        role="separator"
        aria-orientation="horizontal"
        aria-controls="topic-inspector"
        aria-valuenow={Math.round(inspectorHeight)}
        aria-valuemin={INSPECTOR_MIN_HEIGHT}
        aria-valuemax={Math.round(maxHeight)}
        tabIndex={0}
      />
      <TopicInspector topics={topics} />
    </div>
  );
}
