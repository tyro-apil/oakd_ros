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
  
  goalpose_topic = LaunchConfiguration("goalpose_topic")
  goalpose_topic_cmd = DeclareLaunchArgument(
      "goalpose_topic",
      default_value="/ball_pose_topic",
      description="Name of the goal pose topic")
  
  ball_tracking_topic = LaunchConfiguration("ball_tracking_topic")
  ball_tracking_topic_cmd = DeclareLaunchArgument(
      "ball_tracking_topic",
      default_value="/is_ball_tracked",
      description="Name of the ball tracking topic")
  
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
    namespace=namespace, 
    executable='goalpose_node',
    name='goalpose_node',
    parameters=[goalpose_config],
    remappings=[
      ("/odometry/filtered", pose_topic),
      ("/ball_pose_topic", goalpose_topic),
      ("/is_ball_tracked", ball_tracking_topic)
      ]
  )
  
  ld = LaunchDescription()
  
  ld.add_action(namespace_cmd)
  ld.add_action(goalpose_topic_cmd)
  ld.add_action(ball_tracking_topic_cmd)
  ld.add_action(pose_topic_cmd)

  ld.add_action(goalpose_node_cmd)
  return ld