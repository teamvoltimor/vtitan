import { useState, useRef, useEffect } from 'react'
import { RobotState, RosMessageType, type TopicUpdate, type TopicsSnapshot } from '../types'
import './TopicInspector.css'

interface TopicInspectorProps {
  topics: TopicsSnapshot | null
}

function LidarRadarChart({ data }: { data: any }) {
  const canvasRef = useRef<HTMLCanvasElement>(null)
  
  useEffect(() => {
    if (!canvasRef.current || !data.ranges) return
    const ctx = canvasRef.current.getContext('2d')
    if (!ctx) return
    
    const { width, height } = canvasRef.current
    const centerX = width / 2
    const centerY = height / 2
    const scale = Math.min(width, height) / 2 / 2.0 // Max 2.0 meters
    
    ctx.clearRect(0, 0, width, height)
    
    // Draw radar background
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.1)'
    ctx.lineWidth = 1
    for (let r = 0.5; r <= 2.0; r += 0.5) {
      ctx.beginPath()
      ctx.arc(centerX, centerY, r * scale, 0, Math.PI * 2)
      ctx.stroke()
    }
    
    // Draw forward indicator
    ctx.strokeStyle = 'rgba(255, 255, 255, 0.3)'
    ctx.beginPath()
    ctx.moveTo(centerX, centerY)
    ctx.lineTo(centerX, 0)
    ctx.stroke()
    
    const angleMin = data.angle_min ?? 0
    const angleIncrement = data.angle_increment ?? 0.01
    
    // Draw scan points
    data.ranges.forEach((range: number, index: number) => {
      if (range < (data.range_min || 0) || range > (data.range_max || 10)) return
      
      const angle = angleMin + index * angleIncrement
      // Note: canvas y is down, so we subtract
      const x = range * Math.cos(angle)
      const y = range * Math.sin(angle)
      
      const color = range < 0.3 ? '#ff4444' : range < 0.8 ? '#ffaa00' : '#44ff44'
      ctx.fillStyle = color
      ctx.fillRect(
        centerX - y * scale - 1, // ROS x is forward, y is left. For canvas, forward is up (y).
        centerY - x * scale - 1,
        2,
        2
      )
    })
  }, [data])
  
  return (
    <div className="specialized-viz radar-viz">
      <canvas ref={canvasRef} width={200} height={200} />
    </div>
  )
}

function SpeedGauge({ data }: { data: any }) {
  const linearSpeed = data?.linear?.x || 0
  const angularSpeed = data?.angular?.z || 0
  
  const arcPath = (value: number, max: number) => {
    const percent = Math.max(0, Math.min(1, Math.abs(value) / max))
    const angle = Math.PI - (percent * Math.PI)
    const endX = 100 - 80 * Math.cos(Math.PI - angle)
    const endY = 100 - 80 * Math.sin(Math.PI - angle)
    
    return `M 20 100 A 80 80 0 0 1 ${endX} ${endY}`
  }
  
  const needleX = (value: number, max: number) => {
    const percent = Math.max(0, Math.min(1, Math.abs(value) / max))
    return 100 - 70 * Math.cos(percent * Math.PI)
  }
  
  const needleY = (value: number, max: number) => {
    const percent = Math.max(0, Math.min(1, Math.abs(value) / max))
    return 100 - 70 * Math.sin(percent * Math.PI)
  }
  
  return (
    <div className="specialized-viz speed-gauge">
      <svg width="200" height="120" viewBox="0 0 200 120">
        <path
          d="M 20 100 A 80 80 0 0 1 180 100"
          fill="none"
          stroke="rgba(255,255,255,0.1)"
          strokeWidth="12"
        />
        <path
          d={arcPath(linearSpeed, 0.60)}
          fill="none"
          stroke="#4caf50"
          strokeWidth="12"
          strokeLinecap="round"
        />
        <line
          x1="100"
          y1="100"
          x2={needleX(linearSpeed, 0.60)}
          y2={needleY(linearSpeed, 0.60)}
          stroke="#fff"
          strokeWidth="2"
        />
        <circle cx="100" cy="100" r="6" fill="#fff" />
        <text x="100" y="85" textAnchor="middle" fill="#fff" fontSize="20" fontWeight="bold">
          {linearSpeed.toFixed(2)}
        </text>
        <text x="100" y="100" textAnchor="middle" fill="rgba(255,255,255,0.5)" fontSize="10">
          m/s
        </text>
      </svg>
      
      <div className="steering-indicator">
        <span>Steering Rate</span>
        <div className="steering-bar">
          <div 
            className="steering-marker"
            style={{ left: `${50 + (angularSpeed / 2.0) * 50}%` }}
          />
        </div>
        <span>{angularSpeed.toFixed(2)} rad/s</span>
      </div>
    </div>
  )
}

function ImuCompass({ data }: { data: any }) {
  const quaternionToEuler = (q: any) => {
    if (!q) return { roll: 0, pitch: 0, yaw: 0 }
    const { x, y, z, w } = q
    const roll = Math.atan2(2 * (w * x + y * z), 1 - 2 * (x * x + y * y))
    const pitch = Math.asin(Math.max(-1, Math.min(1, 2 * (w * y - z * x))))
    const yaw = Math.atan2(2 * (w * z + x * y), 1 - 2 * (y * y + z * z))
    return { roll, pitch, yaw }
  }

  const { roll, pitch, yaw } = quaternionToEuler(data.orientation)
  const linearAccel = data.linear_acceleration || { x: 0, y: 0, z: 0 }
  const angularVel = data.angular_velocity || { x: 0, y: 0, z: 0 }

  const BarChart = ({ label, value, range, unit }: { label: string, value: number, range: [number, number], unit: string }) => {
    const [min, max] = range
    const percentage = ((value - min) / (max - min)) * 100
    const color = Math.abs(value) > (max - min) * 0.8 ? '#ff4444' : '#4caf50'
    
    return (
      <div className="bar-chart">
        <span className="bar-label">{label}</span>
        <div className="bar-container">
          <div className="bar-background">
            <div className="bar-zero" style={{ left: '50%' }} />
            <div 
              className="bar-fill"
              style={{ 
                width: `${Math.abs(percentage - 50)}%`,
                left: `${Math.min(50, percentage)}%`,
                backgroundColor: color
              }}
            />
          </div>
          <span className="bar-value">{value.toFixed(2)} {unit}</span>
        </div>
      </div>
    )
  }

  return (
    <div className="specialized-viz imu-compass">
      <div className="imu-bars">
        <BarChart label="Accel X" value={linearAccel.x} range={[-10, 10]} unit="m/s²" />
        <BarChart label="Accel Y" value={linearAccel.y} range={[-10, 10]} unit="m/s²" />
        <BarChart label="Accel Z" value={linearAccel.z} range={[-10, 10]} unit="m/s²" />
        
        <BarChart label="Gyro X" value={angularVel.x} range={[-5, 5]} unit="rad/s" />
        <BarChart label="Gyro Y" value={angularVel.y} range={[-5, 5]} unit="rad/s" />
        <BarChart label="Gyro Z" value={angularVel.z} range={[-5, 5]} unit="rad/s" />
      </div>
      
      <div className="orientation-values">
        <span>Roll: {(roll * 180 / Math.PI).toFixed(1)}°</span>
        <span>Pitch: {(pitch * 180 / Math.PI).toFixed(1)}°</span>
        <span>Yaw: {(yaw * 180 / Math.PI).toFixed(1)}°</span>
      </div>
    </div>
  )
}

function MotorDials({ data }: { data: any }) {
  if (!data?.name || !data?.position) return null
  
  const arcPath = (value: number) => {
    // Normalize position from -PI to PI
    let normalized = value % (Math.PI * 2)
    if (normalized < -Math.PI) normalized += Math.PI * 2
    if (normalized > Math.PI) normalized -= Math.PI * 2
    
    // Map to 0-1 percentage
    const percent = (normalized + Math.PI) / (Math.PI * 2)
    const angle = Math.PI - (percent * Math.PI * 2)
    
    const endX = 60 - 50 * Math.cos(Math.PI - angle)
    const endY = 60 - 50 * Math.sin(Math.PI - angle)
    
    // We always draw from bottom (180 deg)
    return `M 60 110 A 50 50 0 ${percent > 0.5 ? 1 : 0} 1 ${endX} ${endY}`
  }

  return (
    <div className="motor-dials specialized-viz">
      {data.name.map((name: string, index: number) => (
        <div key={name} className="motor-dial">
          <svg width="120" height="120" viewBox="0 0 120 120">
            <circle cx="60" cy="60" r="50" fill="rgba(255,255,255,0.05)" />
            <path
              d={arcPath(data.position[index])}
              fill="none"
              stroke="#4caf50"
              strokeWidth="8"
              strokeLinecap="round"
            />
            <line
              x1="60"
              y1="60"
              x2={60 + 40 * Math.cos(data.position[index] - Math.PI / 2)}
              y2={60 + 40 * Math.sin(data.position[index] - Math.PI / 2)}
              stroke="#fff"
              strokeWidth="2"
            />
            <circle cx="60" cy="60" r="5" fill="#fff" />
            <text x="60" y="75" textAnchor="middle" fill="#fff" fontSize="12">
              {((data.position[index] * 180) / Math.PI).toFixed(0)}°
            </text>
          </svg>
          
          <div className="motor-info">
            <strong>{name}</strong>
            <span>Vel: {data.velocity?.[index]?.toFixed(2) ?? 'N/A'} rad/s</span>
            <span>Eff: {data.effort?.[index]?.toFixed(2) ?? 'N/A'} Nm</span>
          </div>
        </div>
      ))}
    </div>
  )
}

function StateDiagram({ data }: { data: any }) {
  const currentState = data?.data || 'UNKNOWN'
  
  const states = [
    { name: RobotState.BOOT_CHECK, x: 50, y: 30 },
    { name: RobotState.READY, x: 150, y: 30 },
    { name: RobotState.RACING, x: 250, y: 30 },
    { name: RobotState.FINISHED, x: 350, y: 30 },
  ]
  
  return (
    <div className="state-diagram specialized-viz">
      <svg width="400" height="80" viewBox="0 0 400 80">
        <line x1="90" y1="30" x2="120" y2="30" stroke="rgba(255,255,255,0.3)" strokeWidth="2" markerEnd="url(#arrow)" />
        <line x1="190" y1="30" x2="220" y2="30" stroke="rgba(255,255,255,0.3)" strokeWidth="2" markerEnd="url(#arrow)" />
        <line x1="290" y1="30" x2="320" y2="30" stroke="rgba(255,255,255,0.3)" strokeWidth="2" markerEnd="url(#arrow)" />
        
        {states.map(state => (
          <g key={state.name}>
            <circle
              cx={state.x}
              cy={state.y}
              r="20"
              fill={currentState === state.name ? '#4caf50' : 'rgba(255,255,255,0.1)'}
              stroke={currentState === state.name ? '#4caf50' : 'rgba(255,255,255,0.3)'}
              strokeWidth="2"
            />
            <text
              x={state.x}
              y={state.y + 35}
              textAnchor="middle"
              fill={currentState === state.name ? '#4caf50' : 'rgba(255,255,255,0.7)'}
              fontSize="10"
              fontWeight={currentState === state.name ? 'bold' : 'normal'}
            >
              {state.name}
            </text>
          </g>
        ))}
        
        <defs>
          <marker id="arrow" markerWidth="10" markerHeight="10" refX="5" refY="5" orient="auto">
            <polygon points="0,0 10,5 0,10" fill="rgba(255,255,255,0.3)" />
          </marker>
        </defs>
      </svg>
    </div>
  )
}

function VisionBoundingBoxes({ data }: { data: any }) {
  if (!data?.detections || data.detections.length === 0) {
    return <div className="specialized-viz">No detections</div>
  }
  
  return (
    <div className="specialized-viz vision-boxes">
      <svg width="320" height="240" viewBox="0 0 640 480" style={{ background: '#111', borderRadius: '4px' }}>
        {data.detections.map((det: any, i: number) => {
          // Fallbacks based on ROS version diffs handled by bridge or raw msg
          const results = det.results?.[0] || {}
          const classId = results.hypothesis?.class_id || results.id || 'unknown'
          const score = results.hypothesis?.score || results.score || 0
          
          const bbox = det.bbox || {}
          const cx = bbox.center?.position?.x || 0
          const cy = bbox.center?.position?.y || 0
          const w = bbox.size_x || 0
          const h = bbox.size_y || 0
          
          const x = cx - w / 2
          const y = cy - h / 2
          
          const color = classId.includes('red') ? '#ff4444' : classId.includes('green') ? '#4caf50' : '#2196f3'
          
          return (
            <g key={i}>
              <rect
                x={x}
                y={y}
                width={w}
                height={h}
                fill="none"
                stroke={color}
                strokeWidth="4"
              />
              <rect x={x} y={y - 20} width={Math.max(100, classId.length * 10)} height="20" fill={color} />
              <text x={x + 5} y={y - 5} fill="#fff" fontSize="14" fontWeight="bold">
                {classId} ({(score * 100).toFixed(0)}%)
              </text>
            </g>
          )
        })}
      </svg>
    </div>
  )
}

function JsonView({ data, level = 0 }: { data: any; level?: number }) {
  if (data === null || data === undefined) {
    return <span className="json-null">null</span>
  }
  
  if (typeof data === 'number') {
    return <span className="json-number">{data.toFixed(4)}</span>
  }
  
  if (typeof data === 'string') {
    return <span className="json-string">"{data}"</span>
  }
  
  if (typeof data === 'boolean') {
    return <span className="json-boolean">{data.toString()}</span>
  }
  
  if (Array.isArray(data)) {
    if (data.length > 10) {
      return (
        <span className="json-array">
          [Array({data.length})] {data.slice(0, 3).map(String).join(', ')}...
        </span>
      )
    }
    return (
      <span className="json-array">
        [{data.map((item, i) => (
          <span key={i}>
            <JsonView data={item} level={level + 1} />
            {i < data.length - 1 && ', '}
          </span>
        ))}]
      </span>
    )
  }
  
  if (typeof data === 'object') {
    const entries = Object.entries(data)
    return (
      <div className="json-object" style={{ marginLeft: `${level * 16}px` }}>
        {entries.map(([key, value]) => (
          <div key={key} className="json-field">
            <span className="json-key">{key}:</span>{' '}
            <JsonView data={value} level={level + 1} />
          </div>
        ))}
      </div>
    )
  }
  
  return <span>{String(data)}</span>
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
          <span className="status-badge offline">No topics</span>
        </div>
        <p className="inspector-empty">Waiting for ROS2 topics...</p>
      </div>
    )
  }
  
  const toggleTopic = (topicName: string) => {
    const newExpanded = new Set(expandedTopics)
    if (newExpanded.has(topicName)) {
      newExpanded.delete(topicName)
    } else {
      newExpanded.add(topicName)
    }
    setExpandedTopics(newExpanded)
  }
  
  const filteredTopics = topics.topics.filter(topic =>
    topic.topicName.toLowerCase().includes(searchFilter.toLowerCase())
  )
  
  const renderTopicVisualization = (topic: TopicUpdate) => {
    if (!visualMode) {
      return <JsonView data={topic.data} />
    }
    
    switch (topic.messageType) {
      case RosMessageType.LASER_SCAN:
        return <LidarRadarChart data={topic.data} />
      case RosMessageType.IMU:
        return <ImuCompass data={topic.data} />
      case RosMessageType.TWIST:
        return <SpeedGauge data={topic.data} />
      case RosMessageType.JOINT_STATE:
        return <MotorDials data={topic.data} />
      case RosMessageType.DETECTION_2D_ARRAY:
        return <VisionBoundingBoxes data={topic.data} />
      case RosMessageType.STRING:
        if (topic.topicName.includes('state')) {
          return <StateDiagram data={topic.data} />
        }
        return <JsonView data={topic.data} />
      default:
        return <JsonView data={topic.data} />
    }
  }
  
  return (
    <div className="topic-inspector">
      <div className="inspector-header">
        <h3>Sensor Dashboard</h3>
        <button
          className="mode-toggle"
          onClick={() => setVisualMode(!visualMode)}
        >
          {visualMode ? '📊 Visual' : '{ } JSON'}
        </button>
        <span className="status-badge online">
          {topics.topics.length} topics
        </span>
      </div>
      
      <input
        type="text"
        className="topic-search"
        placeholder="Filter topics..."
        value={searchFilter}
        onChange={(e) => setSearchFilter(e.target.value)}
      />
      
      <div className="topic-list">
        {filteredTopics.map((topic) => {
          const isExpanded = expandedTopics.has(topic.topicName)
          const age = Date.now() / 1000 - topic.timestamp
          const isStale = age > 2.0
          
          return (
            <div key={topic.topicName} className="topic-item">
              <div
                className="topic-header"
                onClick={() => toggleTopic(topic.topicName)}
              >
                <span className="topic-toggle">{isExpanded ? '▼' : '▶'}</span>
                <div className="topic-info">
                  <strong>{topic.topicName}</strong>
                  <span className="topic-type">{topic.messageType}</span>
                </div>
                <div className="topic-meta">
                  <span className={`topic-rate ${isStale ? 'stale' : ''}`}>
                    {topic.updateRateHz.toFixed(1)} Hz
                  </span>
                  {isStale && <span className="stale-badge">STALE</span>}
                </div>
              </div>
              
              {isExpanded && (
                <div className="topic-data">
                  {renderTopicVisualization(topic)}
                </div>
              )}
            </div>
          )
        })}
      </div>
    </div>
  )
}
