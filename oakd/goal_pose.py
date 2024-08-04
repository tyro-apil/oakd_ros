"""goal_pose.py - goal_pose_node
Publishes goalPose message w.r.t. map frame
"""

import time
from collections import namedtuple
from math import atan2, cos, pi, sin, sqrt
from typing import List, Tuple
import copy

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from geometry_msgs.msg import PoseStamped, TransformStamped
from nav_msgs.msg import Odometry
from oakd_msgs.msg import SpatialBall, SpatialBallArray, StatePose
from rclpy.node import Node
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from rclpy.time import Duration, Time
from scipy.spatial.transform import Rotation as R
from sensor_msgs.msg import Image
from std_msgs.msg import Bool
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from collections import deque


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

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )
    self.balls_baselink_subscriber
    self.img_subscriber = self.create_subscription(
      Image, "image_raw", self.img_callback, qos_profile=image_qos_profile
    )
    self.bridge = CvBridge()
    self.recent_rgb_image = None
    self.target_locked = False
    self.towards_target = False
    self.target_location_queue = deque(maxlen=self.goalpose_consistency_counter)
    self.averaged_target_location = None

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

    self.declare_parameter("enable_dash_at_end", True)
    self.declare_parameter("dash_zone", 0.60)
    self.declare_parameter("dash_distance", 0.20)

    self.declare_parameter("enable_align_zone", False)
    self.declare_parameter("align_distance", 0.20)
    self.declare_parameter("x_align_tolerance", 0.1)

    self.declare_parameter("enable_deadZone", False)
    self.declare_parameter("deadZone_tolerance", 0.2)
    self.declare_parameter("increase_deadZone_x", 0.2)
    self.declare_parameter("backward_distance", 0.30)

    self.declare_parameter("enable_continuous_goalpose", False)
    self.declare_parameter("continuous_goalpose_duration", 0.100)

    self.declare_parameter("enable_incremental_dash", False)
    self.declare_parameter("incremental_dash_roi", [0] * 4)
    self.declare_parameter("x_increment_dash", 0.0)
    self.declare_parameter("y_increment_dash", 0.0)
    self.declare_parameter("roi_match_fraction", 0.0)

    self.declare_parameter("red1_h_low", 0)
    self.declare_parameter("red1_s_low", 100)
    self.declare_parameter("red1_v_low", 40)
    self.declare_parameter("red1_h_high", 15)
    self.declare_parameter("red1_s_high", 255)
    self.declare_parameter("red1_v_high", 235)

    self.declare_parameter("red2_h_low", 165)
    self.declare_parameter("red2_s_low", 115)
    self.declare_parameter("red2_v_low", 65)
    self.declare_parameter("red2_h_high", 185)
    self.declare_parameter("red2_s_high", 255)
    self.declare_parameter("red2_v_high", 210)

    self.declare_parameter("blue1_h_low", 80)
    self.declare_parameter("blue1_s_low", 130)
    self.declare_parameter("blue1_v_low", 30)
    self.declare_parameter("blue1_h_high", 110)
    self.declare_parameter("blue1_s_high", 170)
    self.declare_parameter("blue1_v_high", 90)

    self.declare_parameter("blue2_h_low", 100)
    self.declare_parameter("blue2_s_low", 100)
    self.declare_parameter("blue2_v_low", 50)
    self.declare_parameter("blue2_h_high", 115)
    self.declare_parameter("blue2_s_high", 230)
    self.declare_parameter("blue2_v_high", 230)

    self.declare_parameter("enable_goalpose_lock", False)
    self.declare_parameter("goalpose_consistency_radius", 0.0)
    self.declare_parameter("goalpose_consistency_counter", )

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

    self.__enable_dash_at_end = (
      self.get_parameter("enable_dash_at_end").get_parameter_value().bool_value
    )
    self.dash_zone = self.get_parameter("dash_zone").get_parameter_value().double_value
    self.dash_distance = (
      self.get_parameter("dash_distance").get_parameter_value().double_value
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
    self.__increase_deadZone_x = (
      self.get_parameter("increase_deadZone_x").get_parameter_value().double_value
    )
    self.__backward_distance = (
      self.get_parameter("backward_distance").get_parameter_value().double_value
    )

    self.__enable_continuous_goalpose = (
      self.get_parameter("enable_continuous_goalpose").get_parameter_value().bool_value
    )
    self.continuous_goalpose_duration = (
      self.get_parameter("continuous_goalpose_duration")
      .get_parameter_value()
      .double_value
    )

    self.__enable_incremental_dash = (
      self.get_parameter("enable_incremental_dash").get_parameter_value().bool_value
    )
    self.incremental_dash_roi = (
      self.get_parameter("incremental_dash_roi")
      .get_parameter_value()
      .double_array_value
    )
    self.incremental_dash_roi = [int(i) for i in self.incremental_dash_roi]
    self.x_increment_dash = (
      self.get_parameter("x_increment_dash").get_parameter_value().double_value
    )
    self.y_increment_dash = (
      self.get_parameter("y_increment_dash").get_parameter_value().double_value
    )
    self.roi_match_fraction = (
      self.get_parameter("roi_match_fraction").get_parameter_value().double_value
    )

    self.red1_h_low = (
      self.get_parameter("red1_h_low").get_parameter_value().integer_value
    )
    self.red1_s_low = (
      self.get_parameter("red1_s_low").get_parameter_value().integer_value
    )
    self.red1_v_low = (
      self.get_parameter("red1_v_low").get_parameter_value().integer_value
    )
    self.red1_h_high = (
      self.get_parameter("red1_h_high").get_parameter_value().integer_value
    )
    self.red1_s_high = (
      self.get_parameter("red1_s_high").get_parameter_value().integer_value
    )
    self.red1_v_high = (
      self.get_parameter("red1_v_high").get_parameter_value().integer_value
    )

    self.red2_h_low = (
      self.get_parameter("red2_h_low").get_parameter_value().integer_value
    )
    self.red2_s_low = (
      self.get_parameter("red2_s_low").get_parameter_value().integer_value
    )
    self.red2_v_low = (
      self.get_parameter("red2_v_low").get_parameter_value().integer_value
    )
    self.red2_h_high = (
      self.get_parameter("red2_h_high").get_parameter_value().integer_value
    )
    self.red2_s_high = (
      self.get_parameter("red2_s_high").get_parameter_value().integer_value
    )
    self.red2_v_high = (
      self.get_parameter("red2_v_high").get_parameter_value().integer_value
    )

    self.blue1_h_low = (
      self.get_parameter("blue1_h_low").get_parameter_value().integer_value
    )
    self.blue1_s_low = (
      self.get_parameter("blue1_s_low").get_parameter_value().integer_value
    )
    self.blue1_v_low = (
      self.get_parameter("blue1_v_low").get_parameter_value().integer_value
    )
    self.blue1_h_high = (
      self.get_parameter("blue1_h_high").get_parameter_value().integer_value
    )
    self.blue1_s_high = (
      self.get_parameter("blue1_s_high").get_parameter_value().integer_value
    )
    self.blue1_v_high = (
      self.get_parameter("blue1_v_high").get_parameter_value().integer_value
    )

    self.blue2_h_low = (
      self.get_parameter("blue1_h_low").get_parameter_value().integer_value
    )
    self.blue2_s_low = (
      self.get_parameter("blue1_s_low").get_parameter_value().integer_value
    )
    self.blue2_v_low = (
      self.get_parameter("blue1_v_low").get_parameter_value().integer_value
    )
    self.blue2_h_high = (
      self.get_parameter("blue1_h_high").get_parameter_value().integer_value
    )
    self.blue2_s_high = (
      self.get_parameter("blue1_s_high").get_parameter_value().integer_value
    )
    self.blue2_v_high = (
      self.get_parameter("blue1_v_high").get_parameter_value().integer_value
    )

    self.__enable_goalpose_lock = (
      self.get_parameter("enable_goalpose_lock").get_parameter_value().bool_value
    )
    self.goalpose_consistency_radius = (
      self.get_parameter("goalpose_consistency_radius")
      .get_parameter_value()
      .double_value
    )
    self.goalpose_consistency_counter = (
      self.get_parameter("goalpose_consistency_counter")
      .get_parameter_value()
      .integer_value
    )

  def img_callback(self, img_msg: Image):
    self.recent_rgb_image = self.bridge.imgmsg_to_cv2(img_msg, "bgr8")
    return
    pass

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

          # if self.__lock_far_target:
          #   base2target_vector = (
          #     self.target_ball_location[0] - self.translation_map2base[0],
          #     self.target_ball_location[1] - self.translation_map2base[1],
          #   )
          #   base2target_distance = sqrt(
          #     base2target_vector[0] ** 2 + base2target_vector[1] ** 2
          #   )
          #   if base2target_distance > 1.0:
          #     return
          #   break

    else:
      self.tracked_id, self.target_ball_location = self.get_closest_ball(
        team_colored_balls
      )
      self.set_target_ball(self.tracked_id, self.target_ball_location)

    self.target_location_queue.append(self.target_ball_location)

    if self.__enable_goalpose_lock:
      return
    goalPose_map = self.get_goalpose_map(self.target_ball_location)

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

  def get_goalpose_map(self, target_ball_location):
    goalpose_map = PoseStamped()
    goalpose_map.header.stamp = self.__msg_stamp
    goalpose_map.header.frame_id = "map"

    yaw = round(
      self.get_goalPose_yaw(target_ball_location), self.__decimal_accuracy
    )
    target_map = [0.0] * 3
    target_map[0] = target_ball_location[0] + (
      -self.__x_intake_offset * cos(yaw) + self.__y_intake_offset * sin(yaw)
    )
    target_map[1] = target_ball_location[1] + (
      -self.__x_intake_offset * sin(yaw) - self.__y_intake_offset * cos(yaw)
    )

    if self.__yaw_90 and self.__enable_deadZone:
      if self.is_target_in_deadZone(target_ball_location):
        target_map[0] = self.translation_map2base[0]
        if self.team_color == "blue":
          target_map[1] = self.translation_map2base[1] - self.__backward_distance
        else:
          target_map[1] = self.translation_map2base[1] + self.__backward_distance

    if self.__yaw_90 and self.__enable_align_zone:
      if self.is_target_in_alignZone(target_ball_location):
        if self.team_color == "blue":
          target_map[0] = self.translation_map2base[0] + self.__y_intake_offset
        else:
          target_map[0] = target_ball_location[0] - self.__y_intake_offset
        target_map[1] = self.translation_map2base[1]

    if self.__yaw_90 and self.__enable_dash_at_end:
      if self.is_target_in_dashZone(target_ball_location):
        if self.team_color == "blue":
          target_map[1] = self.translation_map2base[1] + self.dash_distance
        else:
          target_map[1] = self.translation_map2base[1] - self.dash_distance

    if self.__yaw_90 and self.__enable_incremental_dash:
      if self.recent_rgb_image is not None:
        # convert to a hsv frame
        hsv_image = cv2.cvtColor(self.recent_rgb_image, cv2.COLOR_BGR2HSV)

        # get masks from hsv frame
        team_color_mask = self.get_mask(hsv_image, self.team_color)

        # check if team color is within roi
        # get match percentage of team color in roi
        match_percent = self.compute_match_percent(
          hsv_image, tuple(self.incremental_dash_roi), team_color_mask
        )

        # dash in y-direction if match percentage is above threshold
        if match_percent > self.roi_match_fraction:
          if self.team_color == "blue":
            target_map[1] = self.translation_map2base[1] + self.y_increment_dash
          else:
            target_map[1] = self.translation_map2base[1] - self.y_increment_dash

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

    if self.__enable_continuous_goalpose:
      if not self.is_target_in_dashZone():
        return goalpose_map
      cur = time.time()
      while (time.time() - cur) < self.continuous_goalpose_duration:
        self.goalpose_publisher.publish(goalpose_map)

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

  def is_target_in_dashZone(self, target_ball_location):
    if (
      abs(target_ball_location[1] - self.translation_map2base[1]) <= self.dash_zone
    ):
      return True
    return False

  def is_target_in_alignZone(self, target_ball_location):
    if (
      abs(target_ball_location[1] - self.translation_map2base[1])
      <= self.base_fblr[0] + self.__align_distance
    ) and abs(
      target_ball_location[0] - self.translation_map2base[0]
    ) >= self.__x_align_tolerance:
      return True
    return False

  def is_target_in_deadZone(self, target_ball_location):
    if self.team_color == "blue":
      top_left = (
        self.translation_map2base[0] - (self.base_fblr[2] - self.__increase_deadZone_x),
        self.translation_map2base[1] + (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      top_right = (
        self.translation_map2base[0] + (self.base_fblr[3] - self.__increase_deadZone_x),
        self.translation_map2base[1] + (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      if (
        target_ball_location[0] < top_left[0]
        and target_ball_location[1] < top_left[1]
      ) or (
        target_ball_location[0] > top_right[0]
        and target_ball_location[1] < top_right[1]
      ):
        return True
      return False
    else:
      top_left = (
        self.translation_map2base[0] + (self.base_fblr[2] - self.__increase_deadZone_x),
        self.translation_map2base[1] - (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      top_right = (
        self.translation_map2base[0] - (self.base_fblr[3] - self.__increase_deadZone_x),
        self.translation_map2base[1] - (self.base_fblr[0] + self.__deadZone_tolerance),
      )
      if (
        target_ball_location[0] > top_left[0]
        and target_ball_location[1] > top_left[1]
      ) or (
        target_ball_location[0] < top_right[0]
        and target_ball_location[1] > top_right[1]
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

  def get_mask(self, hsv_frame: cv2.Mat, color: str) -> cv2.Mat:
    match color:
      case "red":
        red1_hsv_low = np.array([self.red1_h_low, self.red1_s_low, self.red1_v_low])
        red1_hsv_high = np.array([self.red1_h_high, self.red1_s_high, self.red1_v_high])

        red2_hsv_low = np.array([self.red2_h_low, self.red2_s_low, self.red2_v_low])
        red2_hsv_high = np.array([self.red2_h_high, self.red2_s_high, self.red2_v_high])

        red1_mask = cv2.inRange(hsv_frame, red1_hsv_low, red1_hsv_high)
        red2_mask = cv2.inRange(hsv_frame, red2_hsv_low, red2_hsv_high)
        mask = cv2.bitwise_or(red1_mask, red2_mask)

      case "blue":
        blue1_hsv_low = np.array([self.blue1_h_low, self.blue1_s_low, self.blue1_v_low])
        blue1_hsv_high = np.array(
          [self.blue1_h_high, self.blue1_s_high, self.blue1_v_high]
        )

        blue2_hsv_low = np.array([self.blue2_h_low, self.blue2_s_low, self.blue2_v_low])
        blue2_hsv_high = np.array(
          [self.blue2_h_high, self.blue2_s_high, self.blue2_v_high]
        )

        blue1_mask = cv2.inRange(hsv_frame, blue1_hsv_low, blue1_hsv_high)
        mask = blue1_mask
        # blue2_mask = cv2.inRange(hsv_frame, blue2_hsv_low, blue2_hsv_high)
        # mask = cv2.bitwise_or(blue1_mask, blue2_mask)

      case "purple":
        purple_hsv_low = np.array([120, 50, 50])
        purple_hsv_high = np.array([150, 255, 255])

        mask = cv2.inRange(hsv_frame, purple_hsv_low, purple_hsv_high)

    return mask

  def compute_match_percent(self, hsv_img: cv2.Mat, roi: Tuple, mask: cv2.Mat) -> float:
    x1, y1, x2, y2 = roi
    roi_img = hsv_img[y1:y2, x1:x2]
    roi_mask = mask[y1:y2, x1:x2]

    roi_mask = cv2.bitwise_and(roi_img, roi_img, mask=roi_mask)
    roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_HSV2BGR)
    roi_mask = cv2.cvtColor(roi_mask, cv2.COLOR_BGR2GRAY)

    match_percent = cv2.countNonZero(roi_mask) / (roi_mask.shape[0] * roi_mask.shape[1])
    return match_percent

  def preprocess_mask(self, mask: cv2.Mat) -> cv2.Mat:
    kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (5, 5))
    processed_mask = cv2.dilate(mask, kernel, iterations=2)
    return processed_mask

  def combine_masks(self, mask1: cv2.Mat, mask2: cv2.Mat) -> cv2.Mat:
    combined_mask = cv2.bitwise_or(mask1, mask2)
    return combined_mask

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
    if self.__enable_goalpose_lock:

      if self.target_locked:
        self.target_location_queue.clear()
        self.state_n_goalpose_publisher.publish(self.state_n_goalpose)
        self.goalpose_publisher.publish(self.goalpose_map)
        self.target_publisher.publish(self.target_ball)

        self.check_if_inside_goalpose_radius()
      else:
        self.set_ball_tracking_state(False)
        self.update_state_msg()
        self.state_n_goalpose_publisher.publish(self.state_n_goalpose)
        self.goalpose_publisher.publish(self.goalpose_map)

        copy_target_queue = self.target_location_queue.copy()
        self.check_target_consistency(copy_target_queue)
        self.averaged_target_location = self.get_average_target_location(copy_target_queue)
        if self.consistent_target:
          goalPose_map = self.get_goalpose_map(self.averaged_target_location)
          self.set_goalpose_map(goalPose_map)
          self.set_ball_tracking_state(True)
          self.update_state_msg()

    else:
      self.state_n_goalpose_publisher.publish(self.state_n_goalpose)
      self.goalpose_publisher.publish(self.goalpose_map)
      self.target_publisher.publish(self.target_ball)
      return
  
  def check_if_inside_goalpose_radius(self):
    base2goal_vector = (
      self.goalpose_map.pose.position.x - self.translation_map2base[0],
      self.goalpose_map.pose.position.y - self.translation_map2base[1],
    )
    if (
      (base2goal_vector[0] <= self.goalpose_consistency_radius) 
      and (base2goal_vector[1] <= self.goalpose_consistency_radius)
    ):
      self.target_locked = False
    else:
      self.target_locked = True

  def check_target_consistency(self, queue):
    if len(queue) != self.goalpose_consistency_counter:
      return
    # Convert deque to a list for easy indexing
    data_list = list(queue)
    
    # Check x differences
    for i in range(1, len(data_list)):
        x1, _ = data_list[i-1]
        x2, _ = data_list[i]
        if abs(x2 - x1) > self.goalpose_consistency_radius:
            return False
    
    # Check y differences
    for i in range(1, len(data_list)):
        _, y1 = data_list[i-1]
        _, y2 = data_list[i]
        if abs(y2 - y1) > self.goalpose_consistency_radius:
            return False
    
    return True
  
  def get_average_target_location(self, queue):
    x_sum = 0
    y_sum = 0
    for x, y in queue:
        x_sum += x
        y_sum += y
    
    x_avg = x_sum / len(queue)
    y_avg = y_sum / len(queue)
    return [x_avg, y_avg]

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
