/**
 * src/components/TopicInspector.tsx
 *
 * Topic list + per-topic visualization. Specialized visualizers live in
 * components/visualizers/; this file only handles list state (filter, expand,
 * visual/JSON mode) and staleness.
 */

import { useState } from 'react';
import { TELEMETRY_CONFIG, UI_STRINGS } from '../config';
import type { TopicsSnapshot } from '../types';
import { formatNumber, timestampAgeSeconds } from '../utils/formatting';
import { TopicModal } from './TopicModal';
import { StatusBadge } from './ui';
import { TopicVisualization } from './visualizers';
import './TopicInspector.css';

interface TopicInspectorProps {
  topics: TopicsSnapshot | null;
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
            className={`mode-toggle${visualMode ? ' active' : ''}`}
            onClick={() => setVisualMode(!visualMode)}
          >
            {visualMode ? 'Show JSON' : 'Show Visual'}
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
