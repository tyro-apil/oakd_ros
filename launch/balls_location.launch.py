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
  depth_image_topic = LaunchConfiguration("depth_image_topic")
  depth_image_topic_cmd = DeclareLaunchArgument(
      "depth_image_topic",
      default_value="stereo/depth/raw",
      description="Name of the depth image topic")
  
  camera_info_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'camera_info.yaml'
  )

  base2cam_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'base2cam.yaml'
  )

  #
  # NODES
  #
  spatial_node_cmd=Node(
    package='oakd',
    executable='spatial_node',
    name='spatial_node',
    remappings=[("image_raw", depth_image_topic)],
    parameters=[camera_info_config]
  )

  markers_node_cmd=Node(
    package='oakd',
    executable='markers_node',
    name='markers_node'
  )

  cam2base_node_cmd = Node(
    package='oakd',
    executable='cam2base_node',
    name='cam2base_node',
    parameters=[base2cam_config]
  )

  base2map_node_cmd = Node(
    package='oakd',
    executable='base2map_node',
    name='base2map_node'
  )

  
  ld = LaunchDescription()
  
  ld.add_action(depth_image_topic_cmd)

  ld.add_action(spatial_node_cmd)
  ld.add_action(cam2base_node_cmd)
  ld.add_action(base2map_node_cmd)
  ld.add_action(markers_node_cmd)

  return ld