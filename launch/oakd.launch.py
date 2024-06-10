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
  namespace = '/oak'
  input_image_topic = 'rgb/rect'
  depth_image_topic = 'stereo/depth'
  tracking_topic = 'tracking'
  model = 'oakd_nano.pt'
  tracker = 'custom_tracker.yaml'
  baselink_pose_topic = '/odometry/filtered'
  state_n_goalpose_topic = '/ball_tracking'
  
  cam_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/driver.launch.py']),
    launch_arguments={
      'rgb_image_topic': input_image_topic,
      'depth_image_topic': depth_image_topic,
      'namespace': namespace
    }.items()
    )
  
  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('yolov8_bringup'), 'launch'),
      '/yolov8.launch.py']),
    launch_arguments={
      'namespace': namespace+'/yolo',                    # By default, the namespace is set to 'yolo'
      'input_image_topic': namespace+'/'+input_image_topic,
      'model': os.path.join(get_package_share_directory('yolov8_ros'), 'models', f'{model}'),
      'tracker': os.path.join(get_package_share_directory('yolov8_ros'), 'config', f'{tracker}')
    }.items()
    )
  
  ball_location = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/balls_location.launch.py']),
    launch_arguments={
      'depth_image_topic': depth_image_topic,
      'pose_topic': baselink_pose_topic,
      'namespace': namespace,
      'tracking_topic': namespace+'/yolo/'+tracking_topic
    }.items()
    )

  goalpose = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/goalpose.launch.py']),
    launch_arguments={
      'pose_topic': baselink_pose_topic,
      'namespace': namespace,
      'state_n_goalpose_topic': state_n_goalpose_topic,
    }.items()
    )

  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/transforms.launch.py']),
    launch_arguments={
      'namespace': namespace
    }.items()  
    )
  
  republish = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/republish.launch.py'])
    )

  return LaunchDescription([
    transforms,
    cam_driver,
    yolov8_bringup,
    republish,
    ball_location,
    goalpose
  ])