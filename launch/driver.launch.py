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
  camera_config = os.path.join(
    get_package_share_directory("oakd"), "config", "cam_driver.yaml"
  )

  pose_topic = LaunchConfiguration("pose_topic")
  pose_topic_cmd = DeclareLaunchArgument(
    "pose_topic",
    default_value="/odometry/filtered",
    description="Name of the pose topic of map2base transform",
  )

  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  camera_info_topic = LaunchConfiguration("camera_info_topic")
  camera_info_topic_cmd = DeclareLaunchArgument(
    "camera_info_topic",
    default_value="rgb/camera_info",
    description="Name of the camera image topic",
  )

  rgb_image_topic = LaunchConfiguration("rgb_image_topic")
  rgb_image_topic_cmd = DeclareLaunchArgument(
    "rgb_image_topic",
    default_value="rgb/rect",
    description="Name of the rgb image topic",
  )

  depth_image_topic = LaunchConfiguration("depth_image_topic")
  depth_image_topic_cmd = DeclareLaunchArgument(
    "depth_image_topic",
    default_value="stereo/depth/raw",
    description="Name of the depth image topic",
  )

  #
  # NODES
  #
  driver_node_cmd = Node(
    package="oakd",
    executable="driver_node",
    namespace=namespace,
    name="driver_node",
    parameters=[camera_config],
    remappings=[
      ("rgb/rect", rgb_image_topic),
      ("stereo/depth", depth_image_topic),
      ("/odometry/filtered", pose_topic),
    ],
  )

  camera_info_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="camera_info_node",
    name="camera_info_node",
    parameters=[camera_config],
    remappings=[("camera_info", camera_info_topic)],
  )

  ld = LaunchDescription()
  # Add arguments
  ld.add_action(namespace_cmd)
  ld.add_action(pose_topic_cmd)
  ld.add_action(camera_info_topic_cmd)
  ld.add_action(rgb_image_topic_cmd)
  ld.add_action(depth_image_topic_cmd)
  # Add nodes
  ld.add_action(camera_info_node_cmd)
  ld.add_action(driver_node_cmd)

  return ld
