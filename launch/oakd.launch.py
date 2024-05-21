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

  input_image_topic = 'oak/rgb/image_raw'
  depth_image_topic = 'oak/stereo/image_raw'

  depthai_ros_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('depthai_ros_driver'), 'launch'),
      '/rgbd_pcl.launch.py'])
    )
  cam_driver = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      cam_driver,
    ]
   )
  
  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('yolov8_bringup'), 'launch'),
      '/yolov8.launch.py']),
    launch_arguments={'input_image_topic': input_image_topic}.items()
    )
  yolov8_with_namespace = GroupAction(
     actions=[
         PushRosNamespace('yolo'),
         yolov8_bringup,
      ]
   )
  
  spatial_location = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/spatial.launch.py']),
    launch_arguments={'depth_image_topic': depth_image_topic}.items()
    )
  spatial_location = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      spatial_location,
    ]
  )

  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource([os.path.join(
      get_package_share_directory('oakd'), 'launch'),
      '/transforms.launch.py'])
    )
  transforms = GroupAction(
    actions=[
      PushRosNamespace(namespace),
      transforms,
    ]
  )

  return LaunchDescription([
    cam_driver,
    yolov8_bringup,
    spatial_location,
    transforms
  ])