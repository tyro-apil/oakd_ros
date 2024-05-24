from geometry_msgs.msg import TransformStamped

import rclpy
from rclpy.node import Node

from tf2_ros.static_transform_broadcaster import StaticTransformBroadcaster
import numpy as np
from scipy.spatial.transform import Rotation as R

class StaticFramePublisher(Node):
  def __init__(self):
    super().__init__('static_base2cam_tf_publisher')

    self.tf_static_broadcaster = StaticTransformBroadcaster(self)
    self.declare_parameter("translation", [0.0, 0.0, 0.0])
    self.declare_parameter("ypr", [0.0, 0.0, 0.0])

    self.translation_ = self.get_parameter("translation").get_parameter_value().double_array_value
    self.ypr_ = self.get_parameter("ypr").get_parameter_value().double_array_value

    # Publish static transforms once at startup
    self.make_transforms()
    self.get_logger().info('Static base2cam_optical transform published')

  def make_transforms(self):
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'base_link'
    t.child_frame_id = 'oakd_rgb_camera_optical_frame'

    t.transform.translation.x = float(self.translation_[0])
    t.transform.translation.y = float(self.translation_[1])
    t.transform.translation.z = float(self.translation_[2])

    q_base2cam = R.from_euler("ZYX", self.ypr_, degrees=True).as_quat()
  
    t.transform.rotation.x = q_base2cam[0]
    t.transform.rotation.y = q_base2cam[1]
    t.transform.rotation.z = q_base2cam[2]
    t.transform.rotation.w = q_base2cam[3]

    self.tf_static_broadcaster.sendTransform(t)


def main():

  rclpy.init()
  node = StaticFramePublisher()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()