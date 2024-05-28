import os

from ament_index_python.packages import get_package_share_directory

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.actions import GroupAction
from launch_ros.actions import PushRosNamespace
from launch.substitutions import LaunchConfiguration
from launch.actions import DeclareLaunchArgument

def generate_launch_description():
  namespace = 'oak'

  tracker_yaml_path = '/home/apil/main_ws/src/yolov8_ros/yolov8_ros/config/custom_tracker.yaml'
  input_image_topic = 'rgb/rect'
  depth_image_topic = 'stereo/depth/raw'
  model = 'oakd_nano.pt'
  baselink_pose_topic = 'odometry/filtered'
  
  cam_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/driver.launch.py']),
    launch_arguments={
      'rgb_image_topic': input_image_topic,
      'depth_image_topic': depth_image_topic
    }.items()
    )
  # cam_driver = GroupAction(
  #   actions=[
  #     PushRosNamespace(namespace),
  #     cam_driver,
  #   ]
  #  )
  
  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('yolov8_bringup'), 'launch'),
      '/yolov8.launch.py']),
    launch_arguments={
      'namespace': '',                    # By default, the namespace is set to 'yolo'
      'input_image_topic': input_image_topic,
      'model': model,
      'tracker': tracker_yaml_path
    }.items()
    )
  # breakpoint()
  # yolov8_bringup = GroupAction(
  #   actions=[
  #     PushRosNamespace(namespace),
  #     yolov8_bringup,
  #   ]
  #  )
  
  ball_location = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/balls_location.launch.py']),
    launch_arguments={
      'depth_image_topic': depth_image_topic,
      'pose_topic': baselink_pose_topic,
    }.items()
    )
  # ball_location = GroupAction(
  #   actions=[
  #     PushRosNamespace(namespace),
  #     ball_location,
  #   ]
  # )

  goalpose = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/goalpose.launch.py']),
    launch_arguments={'pose_topic': baselink_pose_topic}.items()
    )
  # goalpose = GroupAction(
  #   actions=[
  #     PushRosNamespace(namespace),
  #     goalpose,
  #   ]
  # )

  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/transforms.launch.py'])
    )
  # transforms = GroupAction(
  #   actions=[
  #     PushRosNamespace(namespace),
  #     transforms,
  #   ]
  # )

  return LaunchDescription([
    transforms,
    cam_driver,
    yolov8_bringup,
    ball_location,
    goalpose
  ])