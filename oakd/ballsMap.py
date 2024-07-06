from collections import namedtuple

import numpy as np
import rclpy
from nav_msgs.msg import Odometry
from oakd_msgs.msg import SpatialBall, SpatialBallArray
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class Base2MapCoordinateTransform(Node):
  def __init__(self):
    super().__init__("base2map_node")

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    XY_limits = namedtuple("XY_limits", "xmin ymin xmax ymax")

    self.__to_frame_rel = "base_link"
    self.__from_frame_rel = "map"
    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    self.declare_parameter("ball_diameter", 0.190)
    self.declare_parameter("clip_ball_xy", False)
    self.declare_parameter("filter_ball_xy", False)
    self.declare_parameter("xy_clip_limits", [0.0] * 4)
    self.declare_parameter("xy_filter_limits", [0.0] * 4)
    self.declare_parameter("set_fixed_z", True)
    self.declare_parameter("decimal_accuracy", 3)

    ## Publisher of ball position data in real world
    self.balls_map_publisher = self.create_publisher(SpatialBallArray, "balls_map", 10)

    self.balls_baselink_subscriber = self.create_subscription(
      SpatialBallArray, "balls_baselink", self.balls_baselink_cb, 10
    )
    self.balls_baselink_subscriber  # prevent unused variable warning

    self.baselink_pose_subscriber = self.create_subscription(
      Odometry,
      "/odometry/filtered",
      self.baselink_pose_callback,
      qos_profile=qos_profile,
    )
    self.baselink_pose_subscriber  # prevent unused variable warning

    self.__decimal_accuracy = (
      self.get_parameter("decimal_accuracy").get_parameter_value().integer_value
    )
    self.__set_fixed_z = (
      self.get_parameter("set_fixed_z").get_parameter_value().bool_value
    )
    self.ball_diameter = (
      self.get_parameter("ball_diameter").get_parameter_value().double_value
    )
    self.__clip_ball_xy = (
      self.get_parameter("clip_ball_xy").get_parameter_value().bool_value
    )
    self.__filter_ball_xy = (
      self.get_parameter("filter_ball_xy").get_parameter_value().bool_value
    )
    xy_clip_limits = (
      self.get_parameter("xy_clip_limits").get_parameter_value().double_array_value
    )
    xy_filter_limits = (
      self.get_parameter("xy_filter_limits").get_parameter_value().double_array_value
    )

    self.xy_clip_limits = XY_limits(*xy_clip_limits)
    self.xy_filter_limits = XY_limits(*xy_filter_limits)

    self.translation_map2base = None
    self.quaternion_map2base = None

    self.proj_map2base = None

    self.get_logger().info("Baselink2map coordinate transformation node started.")

  def balls_baselink_cb(self, msg: SpatialBallArray):
    """Set the balls_map_msg after receiving coordinates of balls w.r.t. baselink"""
    if self.proj_map2base is None:
      return

    balls_map_msg = SpatialBallArray()
    balls_map_msg.header.stamp = msg.header.stamp
    balls_map_msg.header.frame_id = "map"
    # breakpoint()
    for ball in msg.spatial_balls:
      ball_map_msg = SpatialBall()
      ball_map_msg = ball

      ball_baselink_xyz = [ball.position.x, ball.position.y, ball.position.z]
      ball_map_xyz = self.base2map(ball_baselink_xyz)

      if self.__clip_ball_xy:
        ball_map_xyz[0] = np.clip(
          ball_map_xyz[0],
          self.xy_clip_limits.xmin + self.ball_diameter / 2,
          self.xy_clip_limits.xmax - self.ball_diameter / 2,
        )
        ball_map_xyz[1] = np.clip(
          ball_map_xyz[1],
          self.xy_clip_limits.ymin + self.ball_diameter / 2,
          self.xy_clip_limits.ymax - self.ball_diameter / 2,
        )

      ball_map_msg.position.x = round(float(ball_map_xyz[0]), self.__decimal_accuracy)
      ball_map_msg.position.y = round(float(ball_map_xyz[1]), self.__decimal_accuracy)
      ball_map_msg.position.z = round(self.ball_diameter / 2, self.__decimal_accuracy)

      if not self.__set_fixed_z:
        ball_map_msg.position.z = round(float(ball_map_xyz[2]), self.__decimal_accuracy)

      balls_map_msg.spatial_balls.append(ball_map_msg)

    if self.__filter_ball_xy:
      balls_map_msg.spatial_balls = self.filter_balls(balls_map_msg.spatial_balls)

    self.balls_map_publisher.publish(balls_map_msg)
    return

  def filter_balls(self, balls):
    """Filter balls based on the xy limits"""
    filtered_balls = [
      ball
      for ball in balls
      if ball.position.x > self.xy_filter_limits.xmin
      and ball.position.x < self.xy_filter_limits.xmax
      and ball.position.y > self.xy_filter_limits.ymin
      and ball.position.y < self.xy_filter_limits.ymax
    ]
    return filtered_balls

  def base2map(self, baselink_point):
    """Transform the point from base_link frame to map frame"""
    baselink_point.append(1.0)
    baselink_homogeneous_point = np.array(baselink_point, dtype=np.float32)
    map_point_homogeneous = np.dot(
      self.proj_map2base, baselink_homogeneous_point.reshape((-1, 1))
    )

    # dehomogenize
    map_point = map_point_homogeneous / map_point_homogeneous[3]
    return map_point[:3].ravel()

  def baselink_pose_callback(self, pose_msg: Odometry):
    """Updates the pose of baselink w.r.t. map"""
    self.translation_map2base = np.zeros(3)
    self.translation_map2base[0] = round(
      pose_msg.pose.pose.position.x, self.__decimal_accuracy
    )
    self.translation_map2base[1] = round(
      pose_msg.pose.pose.position.y, self.__decimal_accuracy
    )
    self.translation_map2base[2] = round(
      pose_msg.pose.pose.position.z, self.__decimal_accuracy
    )

    self.quaternion_map2base = np.zeros(4)
    self.quaternion_map2base[0] = round(
      pose_msg.pose.pose.orientation.x, self.__decimal_accuracy
    )
    self.quaternion_map2base[1] = round(
      pose_msg.pose.pose.orientation.y, self.__decimal_accuracy
    )
    self.quaternion_map2base[2] = round(
      pose_msg.pose.pose.orientation.z, self.__decimal_accuracy
    )
    self.quaternion_map2base[3] = round(
      pose_msg.pose.pose.orientation.w, self.__decimal_accuracy
    )

    self.proj_map2base = np.eye(4)
    self.proj_map2base[:3, :3] = R.from_quat(self.quaternion_map2base).as_matrix()
    self.proj_map2base[:3, 3] = self.translation_map2base


def main(args=None):
  rclpy.init(args=args)

  base2map_coordinate_tf = Base2MapCoordinateTransform()

  rclpy.spin(base2map_coordinate_tf)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  base2map_coordinate_tf.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
