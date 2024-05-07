#! /usr/bin/env python3

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
      default_value="/image_raw",
      description="Name of the input image topic")
  
  #
  # NODES
  #
  spatial_node_cmd=Node(
    package='oakd',
    executable='spatial_node',
    name='spatial_node',
    remappings=[("image_raw", depth_image_topic)]
  )

  markers_node_cmd=Node(
    package='oakd',
    executable='markers_node',
    name='markers_node'
  )

  
  ld = LaunchDescription()
  
  ld.add_action(depth_image_topic_cmd)

  ld.add_action(spatial_node_cmd)
  ld.add_action(markers_node_cmd)

  return ld