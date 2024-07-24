"""goal_pose.py - goal_pose_node
Publishes goalPose message w.r.t. map frame
"""

from collections import namedtuple
from math import atan2, cos, pi, sin, sqrt
from typing import List

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from oakd_msgs.msg import SpatialBall, SpatialBallArray, StatePose
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from rclpy.time import Duration, Time
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener


class GoalPose(Node):
  def __init__(self):
    super().__init__("goal_pose_node")

    self.declare_params()
    self.read_params()

    self.tf_buffer = Buffer()
    self.tf_listener = TransformListener(self.tf_buffer, self)

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    self.create_timer(self.timer_period_sec, self.publish_state_n_goalpose)

    self.state_n_goalpose_publisher = self.create_publisher(
      StatePose, "/ball_tracker", qos_profile=qos_profile
    )
    self.goalpose_publisher = self.create_publisher(
      PoseStamped, "/ball_pose_topic", qos_profile=qos_profile
    )
    self.target_publisher = self.create_publisher(SpatialBall, "target_ball", 10)
    self.baselink_pose_subscriber = self.create_subscription(
      Odometry,
      "/odometry/filtered",
      self.baselink_pose_callback,
      qos_profile=qos_profile,
    )
    self.baselink_pose_subscriber
    self.balls_baselink_subscriber = self.create_subscription(
      SpatialBallArray, "balls_map", self.balls_msg_received_callback, 10
    )
    self.balls_baselink_subscriber

    self.translation_map2base = None
    self.quaternion_map2base = None
    self.target_ball = SpatialBall()
    self.goalpose_map = PoseStamped()
    self.state_n_goalpose = StatePose()

    self.tracked_id = None
    self.target_ball_location = None
    self.is_ball_tracked = Bool()
    self.__msg_stamp = None

    self.get_logger().info("Goalpose node started")

  def declare_params(self):
    self.declare_parameter("base_fblr", [0.0] * 4)

    self.declare_parameter("timer_period_sec", 0.02)
    self.declare_parameter("team_color", "red")
    self.declare_parameter("ball_diameter", 0.190)

    self.declare_parameter("clamp_goalpose", True)
    self.declare_parameter("yaw_for_corners", True)
    self.declare_parameter("yaw_90", False)
    self.declare_parameter("lock_far_target", False)

    self.declare_parameter("x_intake_offset", 0.60)
    self.declare_parameter("y_intake_offset", 0.15)

    self.declare_parameter("offset_distance", 1.0)
    self.declare_parameter("safe_xy_limits", [0.0] * 4)
    self.declare_parameter("goalpose_limits", [0.0] * 4)
    self.declare_parameter("decimal_accuracy", 3)

    self.declare_parameter("enable_align_zone", False)
    self.declare_parameter("align_distance", 0.20)
    self.declare_parameter("x_align_tolerance", 0.1)

    self.declare_parameter("enable_deadZone", False)
    self.declare_parameter("deadZone_tolerance", 0.2)
    self.declare_parameter("backward_distance", 0.30)

  def read_params(self):
    XY_limits = namedtuple("XY_limits", "xmin ymin xmax ymax")
    # Base polygon w.r.t. base_link -> front, back, left, right
    self.base_fblr = (
      self.get_parameter("base_fblr").get_parameter_value().double_array_value
    )

    self.timer_period_sec = (
      self.get_parameter("timer_period_sec").get_parameter_value().double_value
    )

    self.__decimal_accuracy = (
      self.get_parameter("decimal_accuracy").get_parameter_value().integer_value
    )
    self.safe_xy_limits = (
      self.get_parameter("safe_xy_limits").get_parameter_value().double_array_value
    )
    self.__offset_distance = (
      self.get_parameter("offset_distance").get_parameter_value().double_value
    )
    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.goalpose_limits = (
      self.get_parameter("goalpose_limits").get_parameter_value().double_array_value
    )
    self.safe_xy_limits = XY_limits(*self.safe_xy_limits)
    self.goalpose_limits = XY_limits(*self.goalpose_limits)

    if self.team_color == "red":
      self.goalpose_limits = XY_limits(
        -self.goalpose_limits.xmax,
        -self.goalpose_limits.ymax,
        -self.goalpose_limits.xmin,
        -self.goalpose_limits.ymin,
      )
      self.safe_xy_limits = XY_limits(
        -self.safe_xy_limits.xmax,
        -self.safe_xy_limits.ymax,
        -self.safe_xy_limits.xmin,
        -self.safe_xy_limits.ymin,
      )

    self.__x_intake_offset = (
      self.get_parameter("x_intake_offset").get_parameter_value().double_value
    )
    self.__y_intake_offset = (
      self.get_parameter("y_intake_offset").get_parameter_value().double_value
    )
    self.__clamp_goalpose = (
      self.get_parameter("clamp_goalpose").get_parameter_value().bool_value
    )
    self.__yaw_for_corners = (
      self.get_parameter("yaw_for_corners").get_parameter_value().bool_value
    )
    self.__yaw_90 = self.get_parameter("yaw_90").get_parameter_value().bool_value
    self.__lock_far_target = (
      self.get_parameter("lock_far_target").get_parameter_value().bool_value
    )

    self.__enable_align_zone = (
      self.get_parameter("enable_align_zone").get_parameter_value().bool_value
    )
    self.__align_distance = (
      self.get_parameter("align_distance").get_parameter_value().double_value
    )
    self.__x_align_tolerance = (
      self.get_parameter("x_align_tolerance").get_parameter_value().double_value
    )

    self.__enable_deadZone = (
      self.get_parameter("enable_deadZone").get_parameter_value().bool_value
    )
    self.__deadZone_tolerance = (
      self.get_parameter("deadZone_tolerance").get_parameter_value().double_value
    )
    self.__backward_distance = (
      self.get_parameter("backward_distance").get_parameter_value().double_value
    )

  def baselink_pose_callback(self, pose_msg: Odometry):
    self.translation_map2base = np.zeros(3)
    self.translation_map2base[0] = pose_msg.pose.pose.position.x
    self.translation_map2base[1] = pose_msg.pose.pose.position.y
    self.translation_map2base[2] = pose_msg.pose.pose.position.z

    self.quaternion_map2base = np.zeros(4)
    self.quaternion_map2base[0] = pose_msg.pose.pose.orientation.x
    self.quaternion_map2base[1] = pose_msg.pose.pose.orientation.y
    self.quaternion_map2base[2] = pose_msg.pose.pose.orientation.z
    self.quaternion_map2base[3] = pose_msg.pose.pose.orientation.w

    self.T_map2base = np.eye(4)
    self.T_map2base[:3, :3] = R.from_quat(self.quaternion_map2base).as_matrix()
    self.T_map2base[:3, 3] = self.translation_map2base

    return

  def balls_msg_received_callback(self, SpatialBalls_msg: SpatialBallArray):
    # map2base_tf = self.get_tf("map", "base_link", Time(seconds=0, nanoseconds=0))
    # self.translation_map2base, self.quaternion_map2base = self.compute_pose(map2base_tf)

    if self.translation_map2base is None:
      return

    team_colored_balls = self.filter_balls(SpatialBalls_msg.spatial_balls)
    # self.get_logger().info(
    #   f"detected: {len(SpatialBalls_msg.spatial_balls)} | {self.team_color}: {len(team_colored_balls)}"
    # )

    if len(team_colored_balls) == 0:
      self.set_ball_tracking_state(False)
      self.update_state_msg()
      return

    self.__msg_stamp = SpatialBalls_msg.header.stamp

    if self.tracked_id is not None and self.tracked_id in [
      ball.tracker_id for ball in team_colored_balls
    ]:
      for ball in team_colored_balls:
        if ball.tracker_id == self.tracked_id:
          self.target_ball_location = (ball.position.x, ball.position.y)
          self.set_target_ball(self.tracked_id, self.target_ball_location)

          if self.__lock_far_target:
            base2target_vector = (
              self.target_ball_location[0] - self.translation_map2base[0],
              self.target_ball_location[1] - self.translation_map2base[1],
            )
            base2target_distance = sqrt(
              base2target_vector[0] ** 2 + base2target_vector[1] ** 2
            )
            if base2target_distance > 1.0:
              return
            break

    else:
      self.tracked_id, self.target_ball_location = self.get_closest_ball(
        team_colored_balls
      )
      self.set_target_ball(self.tracked_id, self.target_ball_location)

    goalPose_map = self.get_goalpose_map()

    self.set_goalpose_map(goalPose_map)
    self.set_ball_tracking_state(True)
    self.update_state_msg()
    return

  def filter_balls(self, balls):
    """Filter balls of team color"""
    team_colored_balls = [
      ball
      for ball in balls
      if ball.class_name == self.team_color + "-ball"
      or ball.class_name == self.team_color
    ]
    return team_colored_balls

  def get_closest_ball(self, balls):
    closest_ball = min(
      balls,
      key=lambda ball: self.get_base2ball_distance(ball.position.x, ball.position.y),
    )
    return closest_ball.tracker_id, (closest_ball.position.x, closest_ball.position.y)

  def get_base2ball_distance(self, x_ball_location, y_ball_location):
    base2ball_vector = (
      x_ball_location - self.translation_map2base[0],
      y_ball_location - self.translation_map2base[1],
    )
    return sqrt(base2ball_vector[0] ** 2 + base2ball_vector[1] ** 2)

  def get_goalpose_map(self):
    goalpose_map = PoseStamped()
    goalpose_map.header.stamp = self.__msg_stamp
    goalpose_map.header.frame_id = "map"

    yaw = round(
      self.get_goalPose_yaw(self.target_ball_location), self.__decimal_accuracy
    )
    target_map = [0.0] * 3
    target_map[0] = self.target_ball_location[0] + (
      -self.__x_intake_offset * cos(yaw) + self.__y_intake_offset * sin(yaw)
    )
    target_map[1] = self.target_ball_location[1] + (
      -self.__x_intake_offset * sin(yaw) - self.__y_intake_offset * cos(yaw)
    )

    if self.__yaw_90 and self.__enable_align_zone:
      if self.is_target_in_alignZone():
        if self.team_color == "blue":
          target_map[0] = self.translation_map2base[0] + self.__y_intake_offset
        else:
          target_map[0] = self.target_ball_location[0] - self.__y_intake_offset
        target_map[1] = self.translation_map2base[1]

    if self.__yaw_90 and self.__enable_deadZone:
      if self.is_target_in_deadZone():
        target_map[0] = self.translation_map2base[0]
        if self.team_color == "blue":
          target_map[1] = self.translation_map2base[1] - self.__backward_distance
        else:
          target_map[1] = self.translation_map2base[1] + self.__backward_distance

    if self.__clamp_goalpose:
      target_map = self.clamp_target(target_map)

    goalpose_map.pose.position.x = round(float(target_map[0]), self.__decimal_accuracy)
    goalpose_map.pose.position.y = round(float(target_map[1]), self.__decimal_accuracy)
    goalpose_map.pose.position.z = round(float(target_map[2]), self.__decimal_accuracy)

    q_goalpose = R.from_euler("ZYX", [yaw, 0.0, 0.0]).as_quat()
    goalpose_map.pose.orientation.x = round(q_goalpose[0], self.__decimal_accuracy)
    goalpose_map.pose.orientation.y = round(q_goalpose[1], self.__decimal_accuracy)
    goalpose_map.pose.orientation.z = round(q_goalpose[2], self.__decimal_accuracy)
    goalpose_map.pose.orientation.w = round(q_goalpose[3], self.__decimal_accuracy)

    return goalpose_map

  def get_goalPose_yaw(self, target_ball_location):
    if self.__yaw_90:
      if self.team_color == "blue":
        return pi / 2
      else:
        return -pi / 2

    base2ball_vector = (
      target_ball_location[0] - self.translation_map2base[0],
      target_ball_location[1] - self.translation_map2base[1],
    )

    base2ball_distance = sqrt(base2ball_vector[0] ** 2 + base2ball_vector[1] ** 2)

    if (
      not self.yaw_90
      and self.__yaw_for_corners
      and base2ball_distance < self.__offset_distance
    ):
      if (
        target_ball_location[0] < self.safe_xy_limits.xmin
        and target_ball_location[1] > self.safe_xy_limits.ymin
      ):
        return pi
      elif (
        target_ball_location[0] > self.safe_xy_limits.xmin
        and target_ball_location[1] > self.safe_xy_limits.ymax
      ):
        return pi / 2
      elif (
        target_ball_location[0] > self.safe_xy_limits.xmax
        and target_ball_location[1] < self.safe_xy_limits.ymax
      ):
        return 0.0

    yaw = atan2(base2ball_vector[1], base2ball_vector[0])
    return yaw

  def get_tf(self, from_frame: str, to_frame: str, time: Time) -> TransformStamped:
    try:
      t = self.tf_buffer.lookup_transform(
        from_frame,
        to_frame,
        time=time,
        timeout=Duration(seconds=0.003),
      )
    except TransformException as ex:
      self.get_logger().info(f"Could not transform {from_frame} to {to_frame}: {ex}")
      return None
    return t

  def compute_pose(self, tf: TransformStamped):
    translation = np.zeros(3)
    translation[0] = tf.transform.translation.x
    translation[1] = tf.transform.translation.y
    translation[2] = tf.transform.translation.z

    quaternion = np.zeros(4)
    quaternion[0] = tf.transform.rotation.x
    quaternion[1] = tf.transform.rotation.y
    quaternion[2] = tf.transform.rotation.z
    quaternion[3] = tf.transform.rotation.w

    return translation, quaternion

  def is_target_in_alignZone(self):
    if (
      abs(self.target_ball_location[1] - self.translation_map2base[1])
      < self.base_fblr[0] + self.__align_distance
    ) and abs(
      self.target_ball_location[0] - self.translation_map2base[0]
    ) > self.__x_align_tolerance:
      return True
    return False

  def is_target_in_deadZone(self):
    if self.team_color == "blue":
      top_left = (
        self.translation_map2base[0] - self.base_fblr[2],
        self.translation_map2base[1] + (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      top_right = (
        self.translation_map2base[0] + self.base_fblr[3],
        self.translation_map2base[1] + (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      if (
        self.target_ball_location[0] < top_left[0]
        and self.target_ball_location[1] < top_left[1]
      ) or (
        self.target_ball_location[0] > top_right[0]
        and self.target_ball_location[1] < top_right[1]
      ):
        return True
      return False
    else:
      top_left = (
        self.translation_map2base[0] + self.base_fblr[2],
        self.translation_map2base[1] - (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      top_right = (
        self.translation_map2base[0] - self.base_fblr[3],
        self.translation_map2base[1] - (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      if (
        self.target_ball_location[0] > top_left[0]
        and self.target_ball_location[1] > top_left[1]
      ) or (
        self.target_ball_location[0] < top_right[0]
        and self.target_ball_location[1] > top_right[1]
      ):
        return True
      return False

  def clamp_target(self, target_map):
    clampped_target = target_map

    if target_map[0] < self.goalpose_limits.xmin:
      clampped_target[0] = self.goalpose_limits.xmin
    elif target_map[0] > self.goalpose_limits.xmax:
      clampped_target[0] = self.goalpose_limits.xmax

    if target_map[1] < self.goalpose_limits.ymin:
      clampped_target[1] = self.goalpose_limits.ymin
    elif target_map[1] > self.goalpose_limits.ymax:
      clampped_target[1] = self.goalpose_limits.ymax

    return clampped_target

  def convert_coordinate(self, tf_matrix: np.ndarray, point: List[float]) -> np.ndarray:
    point.append(1.0)
    homogeneous_point = np.array(point, dtype=np.float32)
    converted_point = np.dot(tf_matrix, homogeneous_point.reshape((-1, 1)))
    # dehomogenize
    point = converted_point / converted_point[3]
    return point[:3].ravel()

  def set_goalpose_map(self, goalpose_map):
    self.goalpose_map = goalpose_map
    return

  def set_ball_tracking_state(self, is_tracked):
    self.is_ball_tracked.data = is_tracked
    return

  def set_target_ball_location(self, target_ball_location):
    self.target_ball_location = target_ball_location
    return

  def set_tracked_id(self, tracked_id):
    self.tracked_id = tracked_id
    return

  def set_target_ball(self, target_id, target_location):
    self.target_ball.tracker_id = target_id
    self.target_ball.position.x = target_location[0]
    self.target_ball.position.y = target_location[1]
    self.target_ball.position.z = 0.0
    self.target_ball.class_name = self.team_color
    return

  def update_state_msg(self):
    self.state_n_goalpose.is_tracked.data = self.is_ball_tracked.data
    self.state_n_goalpose.goalpose = self.goalpose_map
    return

  def publish_state_n_goalpose(self):
    self.state_n_goalpose_publisher.publish(self.state_n_goalpose)
    self.goalpose_publisher.publish(self.goalpose_map)
    self.target_publisher.publish(self.target_ball)
    return

  def timer_callback(self) -> None:
    self.publish_state_n_goalpose()


def main(args=None):
  rclpy.init(args=args)

  goal_pose_node = GoalPose()

  rclpy.spin(goal_pose_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  goal_pose_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
