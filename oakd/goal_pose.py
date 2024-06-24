"""goal_pose.py - goal_pose_node
Publishes goalPose message w.r.t. map frame
"""

from collections import namedtuple
from math import atan2, cos, pi, sin, sqrt

import numpy as np
import rclpy
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from oakd_msgs.msg import SpatialBallArray, StatePose
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy
from scipy.spatial.transform import Rotation as R
from std_msgs.msg import Bool


class GoalPose(Node):
  def __init__(self):
    super().__init__("goal_pose_node")

    self.declare_parameter("timer_period_sec", 0.02)
    self.declare_parameter("team_color", "red")
    self.declare_parameter("ball_diameter", 0.190)

    self.declare_parameter("limit_ball_range", True)
    self.declare_parameter("clamp_goalpose", True)
    self.declare_parameter("yaw_for_corners", True)

    self.declare_parameter("x_intake_offset", 0.60)
    self.declare_parameter("y_intake_offset", 0.15)

    self.declare_parameter("ball_xy_limits", [0.0] * 4)
    self.declare_parameter("safe_xy_limits", [0.0] * 4)
    self.declare_parameter("goalpose_limits", [0.0] * 4)

    XY_limits = namedtuple("XY_limits", "xmin ymin xmax ymax")

    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    timer_period_sec = (
      self.get_parameter("timer_period_sec").get_parameter_value().double_value
    )
    self.create_timer(timer_period_sec, self.publish_state_n_goalpose)

    self.state_n_goalpose_publisher = self.create_publisher(
      StatePose, "/ball_tracking", 10
    )
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

    self.ball_xy_limits = (
      self.get_parameter("ball_xy_limits").get_parameter_value().double_array_value
    )
    self.safe_xy_limits = (
      self.get_parameter("safe_xy_limits").get_parameter_value().double_array_value
    )
    self.team_color = (
      self.get_parameter("team_color").get_parameter_value().string_value
    )
    self.goalpose_limits = (
      self.get_parameter("goalpose_limits").get_parameter_value().double_array_value
    )
    self.ball_xy_limits = XY_limits(*self.ball_xy_limits)
    self.safe_xy_limits = XY_limits(*self.safe_xy_limits)
    self.goalpose_limits = XY_limits(*self.goalpose_limits)

    self.__x_intake_offset = (
      self.get_parameter("x_intake_offset").get_parameter_value().double_value
    )
    self.__y_intake_offset = (
      self.get_parameter("y_intake_offset").get_parameter_value().double_value
    )
    self.__limit_ball_range = (
      self.get_parameter("limit_ball_range").get_parameter_value().bool_value
    )
    self.__clamp_goalpose = (
      self.get_parameter("clamp_goalpose").get_parameter_value().bool_value
    )
    self.__yaw_for_corners = (
      self.get_parameter("yaw_for_corners").get_parameter_value().bool_value
    )

    self.translation_map2base = None
    self.quaternion_map2base = None
    self.goalpose_map = PoseStamped()
    self.state_n_goalpose = StatePose()

    self.tracked_id = None
    self.target_ball_location = None
    self.is_ball_tracked = Bool()

    self.get_logger().info("Goalpose node started")

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
    return

  def balls_msg_received_callback(self, SpatialBalls_msg: SpatialBallArray):
    if self.translation_map2base is None:
      self.get_logger().info("Waiting for baselink pose...")
      return

    team_colored_balls = self.filter_balls(SpatialBalls_msg.spatial_balls)
    self.get_logger().info(
      f"deteted: {len(SpatialBalls_msg.spatial_balls)} | {self.team_color}: {len(team_colored_balls)}"
    )

    if len(team_colored_balls) == 0:
      self.set_ball_tracking_state(False)
      self.update_state_msg()
      return

    target_ball_id, target_ball_location = self.get_closest_ball(team_colored_balls)

    goalPose_map = self.get_goalpose_map(target_ball_location)

    self.set_goalpose_map(goalPose_map)
    self.set_ball_tracking_state(True)
    self.set_tracked_id(target_ball_id)
    self.set_target_ball_location(target_ball_location)
    self.update_state_msg()
    return

  def filter_balls(self, balls):
    """Filter balls of team color and within xy limits"""
    team_colored_balls = [ball for ball in balls if ball.class_name == self.team_color]
    if self.__limit_ball_range:
      team_colored_balls = [
        ball
        for ball in team_colored_balls
        if ball.position.x > self.ball_xy_limits.xmin
        and ball.position.x < self.ball_xy_limits.xmax
        and ball.position.y > self.ball_xy_limits.ymin
        and ball.position.y < self.ball_xy_limits.ymax
      ]
    return team_colored_balls

  def get_closest_ball(self, balls):
    closest_ball = min(
      balls,
      key=lambda ball: self.get_base2ball_distance((ball.position.x, ball.position.y)),
    )
    return closest_ball.tracker_id, (closest_ball.position.x, closest_ball.position.y)

  def get_base2ball_distance(self, x_ball_location, y_ball_location):
    base2ball_vector = (
      x_ball_location - self.translation_map2base[0],
      y_ball_location - self.translation_map2base[1],
    )
    return sqrt(base2ball_vector[0] ** 2 + base2ball_vector[1] ** 2)

  def get_goalpose_map(self, target_ball_location):
    goalpose_map = PoseStamped()
    goalpose_map.header.stamp = self.get_clock().now().to_msg()
    goalpose_map.header.frame_id = "map"

    yaw = self.get_goalPose_yaw(target_ball_location)
    target_map = [0.0] * 3
    target_map[0] = target_ball_location[0] + (
      -self.__x_intake_offset * cos(yaw) - self.__y_intake_offset * sin(yaw)
    )
    target_map[1] = target_ball_location[1] + (
      -self.__x_intake_offset * sin(yaw) + self.__y_intake_offset * cos(yaw)
    )

    if self.__clamp_goalpose:
      target_map = self.clamp_target(target_map)

    goalpose_map.pose.position.x = float(target_map[0])
    goalpose_map.pose.position.y = float(target_map[1])
    goalpose_map.pose.position.z = float(target_map[2])

    q_goalpose = R.from_euler("ZYX", [yaw, 0.0, 0.0]).as_quat()
    goalpose_map.pose.orientation.x = q_goalpose[0]
    goalpose_map.pose.orientation.y = q_goalpose[1]
    goalpose_map.pose.orientation.z = q_goalpose[2]
    goalpose_map.pose.orientation.w = q_goalpose[3]

    return goalpose_map

  def get_goalPose_yaw(self, target_ball_location):
    if self.__yaw_for_corners:
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
      elif (
        target_ball_location[0] < self.safe_xy_limits.xmax
        and target_ball_location[1] < self.safe_xy_limits.ymin
      ):
        return -pi / 2
    base_link2ball_vector = (
      target_ball_location[0] - self.translation_map2base[0],
      target_ball_location[1] - self.translation_map2base[1],
    )
    yaw = atan2(base_link2ball_vector[1], base_link2ball_vector[0])
    return yaw

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

  def update_state_msg(self):
    self.state_n_goalpose.is_tracked.data = self.is_ball_tracked.data
    self.state_n_goalpose.goalpose = self.goalpose_map
    return

  def publish_state_n_goalpose(self):
    self.state_n_goalpose_publisher.publish(self.state_n_goalpose)
    return


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
