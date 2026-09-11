import type {
  Detection2DArrayMsg,
  ImuMsg,
  JointStateMsg,
  LaserScanMsg,
  StringMsg,
  TopicUpdate,
  TwistMsg,
} from '../../types';
import { RosMessageType } from '../../types';
import { ImuCompass } from './ImuCompass';
import { JsonView } from './JsonView';
import { LidarRadarChart } from './LidarRadarChart';
import { MotorDials } from './MotorDials';
import { SpeedGauge } from './SpeedGauge';
import { StateDiagram } from './StateDiagram';
import { VisionBoundingBoxes } from './VisionBoundingBoxes';
import { VisionStrip } from './VisionStrip';

export {
  ImuCompass,
  JsonView,
  LidarRadarChart,
  MotorDials,
  SpeedGauge,
  StateDiagram,
  VisionBoundingBoxes,
  VisionStrip,
};

/**
 * Dispatches a topic to its specialized visualizer (or the raw JSON view).
 * The single point where the untyped wire `data` is asserted to a typed message.
 */
export function TopicVisualization({
  topic,
  visualMode,
  expanded,
}: {
  topic: TopicUpdate;
  visualMode: boolean;
  expanded?: boolean;
}) {
  if (!visualMode) {
    return <JsonView data={topic.data} />;
  }

  switch (topic.message_type) {
    case RosMessageType.LASER_SCAN:
      return <LidarRadarChart data={topic.data as unknown as LaserScanMsg} expanded={expanded} />;
    case RosMessageType.IMU:
      return <ImuCompass data={topic.data as unknown as ImuMsg} />;
    case RosMessageType.TWIST:
      return <SpeedGauge data={topic.data as unknown as TwistMsg} />;
    case RosMessageType.JOINT_STATE:
      return <MotorDials data={topic.data as unknown as JointStateMsg} />;
    case RosMessageType.DETECTION_2D_ARRAY:
      return <VisionBoundingBoxes data={topic.data as unknown as Detection2DArrayMsg} />;
    case RosMessageType.STRING:
      if (topic.topic_name.includes('state')) {
        return <StateDiagram data={topic.data as unknown as StringMsg} />;
      }
      return <JsonView data={topic.data} />;
    default:
      return <JsonView data={topic.data} />;
  }
}
