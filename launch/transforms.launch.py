#! /usr/bin/env python3

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
  base2cam_config = os.path.join(
    get_package_share_directory("oakd"), "config", "base2cam.yaml"
  )

  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  base2cam_optical_tf_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="base2cam_optical_tf",
    name="base2cam_optical_tf",
    parameters=[base2cam_config],
  )

  cam_optical2cam_ros_tf_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="cam_optical2cam_ros_tf",
    name="cam_optical2cam_ros_tf",
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(base2cam_optical_tf_node_cmd)
  ld.add_action(cam_optical2cam_ros_tf_node_cmd)

  return ld
