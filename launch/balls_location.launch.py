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
  namespace = LaunchConfiguration("namespace")
  namespace_cmd = DeclareLaunchArgument(
    "namespace", default_value="", description="Name of the namespace"
  )

  pose_topic = LaunchConfiguration("pose_topic")
  pose_topic_cmd = DeclareLaunchArgument(
    "pose_topic",
    default_value="/odometry/filtered",
    description="Name of the pose topic of map2base transform",
  )

  depth_image_topic = LaunchConfiguration("depth_image_topic")
  depth_image_topic_cmd = DeclareLaunchArgument(
    "depth_image_topic",
    default_value="stereo/depth/raw",
    description="Name of the depth image topic",
  )

  tracking_topic = LaunchConfiguration("tracking_topic")
  tracking_topic_cmd = DeclareLaunchArgument(
    "tracking_topic",
    default_value="yolo/tracking",
    description="Name of the tracking topic",
  )

  camera_config = os.path.join(
    get_package_share_directory("oakd"), "config", "cam_driver.yaml"
  )

  common_config = os.path.join(
    get_package_share_directory("oakd"), "config", "common.yaml"
  )

  spatial_location_config = os.path.join(
    get_package_share_directory("oakd"), "config", "spatial_location.yaml"
  )

  base2cam_config = os.path.join(
    get_package_share_directory("oakd"), "config", "base2cam.yaml"
  )

  #
  # NODES
  #
  spatial_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="spatial_node",
    name="spatial_node",
    remappings=[("stereo/depth", depth_image_topic), ("yolo/tracking", tracking_topic)],
    parameters=[camera_config, spatial_location_config],
  )

  markers_node_cmd = Node(
    package="oakd", namespace=namespace, executable="markers_node", name="markers_node"
  )

  cam2base_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="cam2base_node",
    name="cam2base_node",
    parameters=[base2cam_config, common_config],
  )

  base2map_node_cmd = Node(
    package="oakd",
    namespace=namespace,
    executable="base2map_node",
    name="base2map_node",
    remappings=[("/odometry/filtered", pose_topic)],
  )

  ld = LaunchDescription()

  ld.add_action(namespace_cmd)
  ld.add_action(tracking_topic_cmd)
  ld.add_action(pose_topic_cmd)
  ld.add_action(depth_image_topic_cmd)

  ld.add_action(spatial_node_cmd)
  ld.add_action(cam2base_node_cmd)
  ld.add_action(base2map_node_cmd)
  ld.add_action(markers_node_cmd)

  return ld
