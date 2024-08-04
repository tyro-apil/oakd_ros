#! /usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  #
  # ARGS
  #
  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  input_image_topic = LaunchConfiguration("input_image_topic")
  input_image_topic_cmd = DeclareLaunchArgument(
    "input_image_topic", default_value="image_raw", description="Name of the namespace"
  )

  pose_topic = LaunchConfiguration("pose_topic")
  pose_topic_cmd = DeclareLaunchArgument(
    "pose_topic",
    default_value="/odometry/filtered",
    description="Name of the pose topic of map2base transform",
  )

  ball_goalpose_topic = LaunchConfiguration("ball_goalpose_topic")
  ball_goalpose_topic_cmd = DeclareLaunchArgument(
    "ball_goalpose_topic",
    default_value="/ball_pose_topic",
  )

  state_n_goalpose_topic = LaunchConfiguration("state_n_goalpose_topic")
  state_n_goalpose_topic_cmd = DeclareLaunchArgument(
    "state_n_goalpose_topic",
    default_value="/ball_tracker",
    description="Name of the combined tracking state",
  )

  goalpose_config = os.path.join(
    get_package_share_directory("oakd"), "config", "goalpose_config.yaml"
  )

  hsv_config = os.path.join(
    get_package_share_directory("silo"), "config", "check_top.yaml"
  )

  common_config = os.path.join(
    get_package_share_directory("robot"), "config", "common.yaml"
  )

  base_polygon_config = os.path.join(
    get_package_share_directory("robot"), "config", "shape.yaml"
  )

  #
  # NODES
  #
  goalpose_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="goalpose_node",
    name="goalpose_node",
    parameters=[goalpose_config, common_config, base_polygon_config, hsv_config],
    remappings=[
      ("/odometry/filtered", pose_topic),
      ("/ball_tracker", state_n_goalpose_topic),
      ("/ball_pose_topic", ball_goalpose_topic),
      ("image_raw", input_image_topic),
    ],
  )

  target_marker_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="target_marker_node",
    name="target_marker_node",
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(input_image_topic_cmd)
  ld.add_action(ball_goalpose_topic_cmd)
  ld.add_action(state_n_goalpose_topic_cmd)
  ld.add_action(pose_topic_cmd)

  ld.add_action(goalpose_node_cmd)
  ld.add_action(target_marker_node_cmd)
  return ld
