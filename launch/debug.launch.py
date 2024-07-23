#! /usr/bin/env python3

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
    "input_image_topic",
    default_value="image_raw",
  )

  debug_image_topic = LaunchConfiguration("debug_image_topic")
  debug_image_topic_cmd = DeclareLaunchArgument(
    "debug_image_topic",
    default_value="dbg_image",
  )

  #
  # NODES
  #
  capture_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="capture_node",
    name="capture_node",
    remappings=[
      ("image_raw", input_image_topic),
      ("dbg_image", debug_image_topic),
    ],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(input_image_topic_cmd)
  ld.add_action(debug_image_topic_cmd)

  ld.add_action(capture_node_cmd)
  return ld
