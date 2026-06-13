/**
 * src/components/TopicInspector.tsx
 *
 * Topic list + per-topic visualization. Specialized visualizers live in
 * components/visualizers/; this file only handles list state (filter, expand,
 * visual/JSON mode) and staleness.
 */

import { useState } from 'react'
import type { TopicsSnapshot } from '../types'
import { formatNumber, timestampAgeSeconds } from '../utils/formatting'
import { TELEMETRY_CONFIG, UI_STRINGS } from '../config'
import { TopicVisualization } from './visualizers'
import { StatusBadge } from './ui'
import './TopicInspector.css'

interface TopicInspectorProps {
  topics: TopicsSnapshot | null
}

export function TopicInspector({ topics }: TopicInspectorProps) {
  const [expandedTopics, setExpandedTopics] = useState<Set<string>>(new Set())
  const [searchFilter, setSearchFilter] = useState('')
  const [visualMode, setVisualMode] = useState(false)

  if (!topics || topics.topics.length === 0) {
    return (
      <div className="topic-inspector">
        <div className="inspector-header">
          <h3>Topic Inspector</h3>
          <StatusBadge online={false} label="No topics" />
        </div>
        <p className="inspector-empty">Waiting for ROS2 topics…</p>
      </div>
    )
  }

  const toggleTopic = (topicName: string) => {
    const next = new Set(expandedTopics)
    if (next.has(topicName)) next.delete(topicName)
    else next.add(topicName)
    setExpandedTopics(next)
  }

  const filteredTopics = topics.topics.filter((topic) =>
    topic.topic_name.toLowerCase().includes(searchFilter.toLowerCase())
  )

  return (
    <div className="topic-inspector">
      <div className="inspector-header">
        <h3>Sensor Dashboard</h3>
        <button
          className="mode-toggle"
          onClick={() => setVisualMode(!visualMode)}
          aria-pressed={visualMode}
        >
          {visualMode ? '📊 Visual' : '{ } JSON'}
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
          const isExpanded = expandedTopics.has(topic.topic_name)
          const isStale =
            timestampAgeSeconds(topic.timestamp) >
            TELEMETRY_CONFIG.TOPIC_STALENESS_THRESHOLD_SECONDS

          return (
            <div key={topic.topic_name} className="topic-item">
              <div className="topic-header" onClick={() => toggleTopic(topic.topic_name)}>
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
                </div>
              </div>

              {isExpanded && (
                <div className="topic-data">
                  <TopicVisualization topic={topic} visualMode={visualMode} />
                </div>
              )}
            </div>
          )
        })}
      </div>
    </div>
  )
}
