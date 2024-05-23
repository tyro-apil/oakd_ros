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
  goalpose_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'goalpose_config.yaml'
  )

  #
  # NODES
  #
  goalpose_node_cmd=Node(
    package='oakd',
    executable='goalpose_node',
    name='goalpose_node',
    parameters=[goalpose_config]
  )
  
  ld = LaunchDescription()
  
  ld.add_action(goalpose_node_cmd)
  return ld