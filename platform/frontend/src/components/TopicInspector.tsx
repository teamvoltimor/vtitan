/**
 * src/components/TopicInspector.tsx
 *
 * Topic list + per-topic visualization. Specialized visualizers live in
 * components/visualizers/; this file only handles list state (filter, expand,
 * visual/JSON mode) and staleness.
 */

import { useState, useEffect } from 'react';
import { createPortal } from 'react-dom';
import type { TopicsSnapshot, TopicUpdate } from '../types';
import { formatNumber, timestampAgeSeconds } from '../utils/formatting';
import { TELEMETRY_CONFIG, UI_STRINGS } from '../config';
import { TopicVisualization } from './visualizers';
import { StatusBadge } from './ui';
import './TopicInspector.css';

interface TopicInspectorProps {
  topics: TopicsSnapshot | null;
}

interface TopicModalProps {
  topic: TopicUpdate;
  onClose: () => void;
}

function TopicModal({ topic, onClose }: TopicModalProps) {
  const [modalVisualMode, setModalVisualMode] = useState(false);

  useEffect(() => {
    const onKey = (e: KeyboardEvent) => {
      if (e.key === 'Escape') onClose();
    };
    document.addEventListener('keydown', onKey);
    return () => document.removeEventListener('keydown', onKey);
  }, [onClose]);

  return createPortal(
    // biome-ignore lint/a11y/noStaticElementInteractions: backdrop click-to-close; Escape key (handled above) is the keyboard equivalent
    // biome-ignore lint/a11y/useKeyWithClickEvents: backdrop click-to-close; Escape key (handled above) is the keyboard equivalent
    <div className="topic-modal-overlay" onClick={onClose}>
      {/* biome-ignore lint/a11y/useKeyWithClickEvents: only stops the backdrop's click from closing the modal, not itself interactive */}
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

export function TopicInspector({ topics }: TopicInspectorProps) {
  const [expandedTopics, setExpandedTopics] = useState<Set<string>>(new Set());
  const [searchFilter, setSearchFilter] = useState('');
  const [visualMode, setVisualMode] = useState(false);
  const [modalTopicName, setModalTopicName] = useState<string | null>(null);

  if (!topics || topics.topics.length === 0) {
    return (
      <div className="topic-inspector">
        <div className="inspector-header">
          <h3>Topic Inspector</h3>
          <StatusBadge online={false} label="No topics" />
        </div>
        <p className="inspector-empty">Waiting for ROS2 topics…</p>
      </div>
    );
  }

  const toggleTopic = (topicName: string) => {
    const next = new Set(expandedTopics);
    if (next.has(topicName)) next.delete(topicName);
    else next.add(topicName);
    setExpandedTopics(next);
  };

  const filteredTopics = topics.topics.filter((topic) =>
    topic.topic_name.toLowerCase().includes(searchFilter.toLowerCase())
  );

  const modalTopic = modalTopicName
    ? (topics.topics.find((t) => t.topic_name === modalTopicName) ?? null)
    : null;

  return (
    <>
      <div className="topic-inspector">
        <div className="inspector-header">
          <h3>Sensor Dashboard</h3>
          <button
            type="button"
            className="mode-toggle"
            onClick={() => setVisualMode(!visualMode)}
            aria-pressed={visualMode}
          >
            {visualMode ? 'Visual' : 'JSON'}
          </button>
          <StatusBadge online label={`${topics.topics.length} topics`} />
        </div>

        <input
          type="text"
          className="topic-search"
          placeholder={UI_STRINGS.FILTER_TOPICS}
          aria-label={UI_STRINGS.FILTER_TOPICS}
          value={searchFilter}
          onChange={(e) => setSearchFilter(e.target.value)}
        />

        <div className="topic-list">
          {filteredTopics.map((topic) => {
            const isExpanded = expandedTopics.has(topic.topic_name);
            const isStale =
              timestampAgeSeconds(topic.timestamp) >
              TELEMETRY_CONFIG.TOPIC_STALENESS_THRESHOLD_SECONDS;

            return (
              <div key={topic.topic_name} className="topic-item">
                {/* biome-ignore lint/a11y/useSemanticElements: can't be a <button>; it contains a nested "expand" <button> and buttons cannot nest */}
                <div
                  className="topic-header"
                  role="button"
                  tabIndex={0}
                  onClick={() => toggleTopic(topic.topic_name)}
                  onKeyDown={(e) => {
                    if (e.key === 'Enter' || e.key === ' ') {
                      e.preventDefault();
                      toggleTopic(topic.topic_name);
                    }
                  }}
                  aria-expanded={isExpanded}
                >
                  <span className="topic-toggle">{isExpanded ? '▼' : '▶'}</span>
                  <div className="topic-info">
                    <strong>{topic.topic_name}</strong>
                    <span className="topic-type">{topic.message_type}</span>
                  </div>
                  <div className="topic-meta">
                    <span className={`topic-rate ${isStale ? 'stale' : ''}`}>
                      {formatNumber(topic.update_rate_hz, { decimals: 1, unit: 'Hz' })}
                    </span>
                    {isStale && <span className="stale-badge">STALE</span>}
                    <button
                      type="button"
                      className="topic-expand-btn"
                      onClick={(e) => {
                        e.stopPropagation();
                        setModalTopicName(topic.topic_name);
                      }}
                      title="Open in full view"
                      aria-label={`Expand ${topic.topic_name} in full view`}
                    >
                      ⤢
                    </button>
                  </div>
                </div>

                {isExpanded && (
                  <div className="topic-data">
                    <TopicVisualization topic={topic} visualMode={visualMode} />
                  </div>
                )}
              </div>
            );
          })}
        </div>
      </div>

      {modalTopic && <TopicModal topic={modalTopic} onClose={() => setModalTopicName(null)} />}
    </>
  );
}
