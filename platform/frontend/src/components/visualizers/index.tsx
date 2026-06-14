import type {
  TopicUpdate,
  LaserScanMsg,
  TwistMsg,
  ImuMsg,
  JointStateMsg,
  StringMsg,
  Detection2DArrayMsg,
} from '../../types'
import { RosMessageType } from '../../types'
import { LidarRadarChart } from './LidarRadarChart'
import { SpeedGauge } from './SpeedGauge'
import { ImuCompass } from './ImuCompass'
import { MotorDials } from './MotorDials'
import { StateDiagram } from './StateDiagram'
import { VisionBoundingBoxes } from './VisionBoundingBoxes'
import { JsonView } from './JsonView'

export {
  LidarRadarChart,
  SpeedGauge,
  ImuCompass,
  MotorDials,
  StateDiagram,
  VisionBoundingBoxes,
  JsonView,
}

/**
 * Dispatches a topic to its specialized visualizer (or the raw JSON view).
 * The single point where the untyped wire `data` is asserted to a typed message.
 */
export function TopicVisualization({
  topic,
  visualMode,
  expanded,
}: {
  topic: TopicUpdate
  visualMode: boolean
  expanded?: boolean
}) {
  if (!visualMode) {
    return <JsonView data={topic.data} />
  }

  switch (topic.message_type) {
    case RosMessageType.LASER_SCAN:
      return <LidarRadarChart data={topic.data as unknown as LaserScanMsg} expanded={expanded} />
    case RosMessageType.IMU:
      return <ImuCompass data={topic.data as unknown as ImuMsg} />
    case RosMessageType.TWIST:
      return <SpeedGauge data={topic.data as unknown as TwistMsg} />
    case RosMessageType.JOINT_STATE:
      return <MotorDials data={topic.data as unknown as JointStateMsg} />
    case RosMessageType.DETECTION_2D_ARRAY:
      return <VisionBoundingBoxes data={topic.data as unknown as Detection2DArrayMsg} />
    case RosMessageType.STRING:
      if (topic.topic_name.includes('state')) {
        return <StateDiagram data={topic.data as unknown as StringMsg} />
      }
      return <JsonView data={topic.data} />
    default:
      return <JsonView data={topic.data} />
  }
}
