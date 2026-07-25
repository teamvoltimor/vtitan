import { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import type { TopicUpdate } from '../types';
import { TopicVisualization } from './visualizers';
import './TopicInspector.css';

interface TopicModalProps {
  topic: TopicUpdate;
  onClose: () => void;
}

export function TopicModal({ topic, onClose }: TopicModalProps) {
  const [modalVisualMode, setModalVisualMode] = useState(false);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [onClose]);

  return createPortal(
    <div className="topic-modal-overlay" onClick={onClose}>
      <div
        className="topic-modal"
        onClick={(e) => e.stopPropagation()}
        role="dialog"
        aria-modal="true"
        aria-label={topic.topic_name}
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
