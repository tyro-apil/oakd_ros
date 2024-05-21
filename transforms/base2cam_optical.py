from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster

T_BASE2CAM = [0.0, 0.0, 0.0]
QUAT_BASE2CAM = [0.0, 0.0, 0.0, 1.0]

class StaticFramePublisher(Node):
  def __init__(self):
    super().__init__('static_base2cam_tf_publisher')

    self.tf_static_broadcaster = StaticTransformBroadcaster(self)

    # Publish static transforms once at startup
    self.make_transforms()
    self.get_logger().info('Static base2cam_optical transform published')

  def make_transforms(self):
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'oakd_rgb_camera_optical_frame'

    t.transform.translation.x = float(T_BASE2CAM[0])
    t.transform.translation.y = float(T_BASE2CAM[1])
    t.transform.translation.z = float(T_BASE2CAM[2])
  
    t.transform.rotation.x = QUAT_BASE2CAM[0]
    t.transform.rotation.y = QUAT_BASE2CAM[1]
    t.transform.rotation.z = QUAT_BASE2CAM[2]
    t.transform.rotation.w = QUAT_BASE2CAM[3]

    self.tf_static_broadcaster.sendTransform(t)


def main():

  rclpy.init()
  node = StaticFramePublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()