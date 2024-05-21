#! /usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory


from launch import LaunchDescription
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument


def generate_launch_description():
  #
  # ARGS
  #
  camera_info_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'camera_info.yaml'
  )

  rgb_image_topic = LaunchConfiguration("rgb_image_topic")
  rgb_image_topic_cmd = DeclareLaunchArgument(
      "rgb_image_topic",
      default_value="rgb/raw",
      description="Name of the rgb image topic")
  
  depth_image_topic = LaunchConfiguration("depth_image_topic")
  depth_image_topic_cmd = DeclareLaunchArgument(
      "depth_image_topic",
      default_value="stereo/depth/raw",
      description="Name of the depth image topic")

  #
  # NODES
  #
  driver_node_cmd=Node(
    package='oakd',
    executable='driver_node',
    name='driver_node',
    remappings=[
      ("rgb/raw", rgb_image_topic),
      ("stereo/depth/raw", depth_image_topic)
    ]
  )

  camera_info_node_cmd=Node(
    package='oakd',
    executable='camera_info_node',
    name='camera_info_node',
    parameters=[camera_info_config]
  )
  
  ld = LaunchDescription()
  # Add arguments
  ld.add_action(rgb_image_topic_cmd)
  ld.add_action(depth_image_topic_cmd)
  # Add nodes
  ld.add_action(camera_info_node_cmd)
  ld.add_action(driver_node_cmd)

  return ld