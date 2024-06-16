#! /usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
  #
  # NODES
  #
  republish_node_cmd = Node(
    package="image_transport",
    executable="republish",
    name="republish_node",
    output="screen",
    remappings=[
      ("in", "/oak/yolo/dbg_image"),
      ("out/compressed", "/oak/yolo/dbg_image/compressed"),
    ],
    arguments=["raw", "compressed"],
  )

  ld = LaunchDescription()
  # Add nodes
  ld.add_action(republish_node_cmd)

  return ld
