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
  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
      "namespace",
      default_value="",
      description="Name of the namespace")
  
  pose_topic = LaunchConfiguration("pose_topic")
  pose_topic_cmd = DeclareLaunchArgument(
      "pose_topic",
      default_value="/odometry/filtered",
      description="Name of the pose topic of map2base transform")
  
  state_n_goalpose_topic = LaunchConfiguration("state_n_goalpose_topic")
  state_n_goalpose_topic_cmd = DeclareLaunchArgument(
      "state_n_goalpose_topic",
      default_value="/ball_tracking",
      description="Name of the combined tracking state")
  
  goalpose_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'goalpose_config.yaml'
  )

  common_config = os.path.join(
    get_package_share_directory('oakd'),
    'config',
    'common.yaml'
  )

  #
  # NODES
  #
  goalpose_node_cmd=Node(
    package='oakd',
    namespace=namespace, 
    executable='goalpose_node',
    name='goalpose_node',
    parameters=[goalpose_config, common_config],
    remappings=[
      ("/odometry/filtered", pose_topic),
      ("/ball_tracking", state_n_goalpose_topic),
      ]
  )
  
  ld = LaunchDescription()
  
  ld.add_action(namespace_cmd)
  ld.add_action(state_n_goalpose_topic_cmd)
  ld.add_action(pose_topic_cmd)

  ld.add_action(goalpose_node_cmd)
  return ld