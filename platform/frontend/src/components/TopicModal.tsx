import { useState, useEffect, useRef } from 'react';
import { createPortal } from 'react-dom';
import type { TopicUpdate } from '../types';
import { TopicVisualization } from './visualizers';
import './TopicInspector.css';

interface TopicModalProps {
  topic: TopicUpdate;
  onClose: () => void;
}

const FOCUSABLE_SELECTOR =
  'a[href], button:not([disabled]), textarea, input, select, [tabindex]:not([tabindex="-1"])';

export function TopicModal({ topic, onClose }: TopicModalProps) {
  const [modalVisualMode, setModalVisualMode] = useState(false);
  const modalRef = useRef<HTMLDivElement>(null);

  // Focus trap + restore: move focus into the modal on open, cycle Tab/Shift+Tab
  // within it, and return focus to whatever triggered it on close. Without this,
  // Tab escapes to the page behind the modal and focus is lost on close.
  useEffect(() => {
    const previouslyFocused = document.activeElement as HTMLElement | null;
    modalRef.current?.focus();

    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') {
        onClose();
        return;
      }
      if (e.key !== 'Tab' || !modalRef.current) return;

      const focusable = Array.from(
        modalRef.current.querySelectorAll<HTMLElement>(FOCUSABLE_SELECTOR)
      );
      if (focusable.length === 0) return;

      const first = focusable[0];
      const last = focusable[focusable.length - 1];
      if (e.shiftKey && document.activeElement === first) {
        e.preventDefault();
        last.focus();
      } else if (!e.shiftKey && document.activeElement === last) {
        e.preventDefault();
        first.focus();
      }
    };

    document.addEventListener('keydown', onKey);
    return () => {
      document.removeEventListener('keydown', onKey);
      previouslyFocused?.focus();
    };
  }, [onClose]);

  return createPortal(
    // The overlay is a mouse-only convenience dismiss target; keyboard users
    // have the Escape handler above and the visible Close button below.
    // biome-ignore lint/a11y/noStaticElementInteractions: intentional click-outside-to-dismiss; keyboard users have Escape + the Close button
    // biome-ignore lint/a11y/useKeyWithClickEvents: same as above — Escape + the Close button are the keyboard equivalents
    <div className="topic-modal-overlay" onClick={onClose}>
      {/* biome-ignore lint/a11y/useKeyWithClickEvents: onClick here only stops event propagation to the overlay, not a dismiss/activate action */}
      <div
        ref={modalRef}
        className="topic-modal"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-label={topic.topic_name}
        tabIndex={-1}
      >
        <div className="topic-modal-header">
          <div className="topic-modal-title">
            <strong>{topic.topic_name}</strong>
            <span className="topic-type">{topic.message_type}</span>
          </div>
          <div className="topic-modal-controls">
            <button
              type="button"
              className="mode-toggle"
              onClick={() => setModalVisualMode(!modalVisualMode)}
              aria-pressed={modalVisualMode}
            >
              {modalVisualMode ? 'Visual' : 'JSON'}
            </button>
            <button type="button" className="modal-close" onClick={onClose} aria-label="Close">
              ✕
            </button>
          </div>
        </div>
        <div className="topic-modal-body">
          <TopicVisualization topic={topic} visualMode={modalVisualMode} expanded />
        </div>
      </div>
    </div>,
    document.body
  );
}
