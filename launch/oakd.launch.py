import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
  DeclareLaunchArgument,
  EmitEvent,
  ExecuteProcess,
  IncludeLaunchDescription,
  LogInfo,
  RegisterEventHandler,
  TimerAction,
)
from launch.conditions import IfCondition
from launch.event_handlers import (
  OnExecutionComplete,
  OnProcessExit,
  OnProcessIO,
  OnProcessStart,
  OnShutdown,
)
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
  EnvironmentVariable,
  FindExecutable,
  LaunchConfiguration,
  LocalSubstitution,
  PythonExpression,
)


def generate_launch_description():
  namespace = "/oak"
  input_image_topic = "rgb/rect"
  depth_image_topic = "stereo/depth"
  debug_image_topic = "dbg_image"
  tracking_topic = "tracking"
  model = "close_intake.pt"
  tracker = "custom_tracker.yaml"
  baselink_pose_topic = "/odometry/filtered"
  state_n_goalpose_topic = "/ball_tracker"
  ball_goalpose_topic = "/ball_pose_topic"
  device = "cuda:0"
  iou = "0.8"

  cam_driver = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory("oakd"), "launch/driver.launch.py")]
    ),
    launch_arguments={
      "rgb_image_topic": input_image_topic,
      "depth_image_topic": depth_image_topic,
      "namespace": namespace,
    }.items(),
  )

  yolov8_bringup = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(
          get_package_share_directory("yolov8_bringup"), "launch/yolov8.launch.py"
        )
      ]
    ),
    launch_arguments={
      "namespace": namespace + "/yolo",  # By default, the namespace is set to 'yolo'
      "input_image_topic": namespace + "/" + input_image_topic,
      "model": os.path.join(get_package_share_directory("robot"), "models", f"{model}"),
      "tracker": os.path.join(
        get_package_share_directory("robot"), "config", f"{tracker}"
      ),
      "device": device,
      "iou": iou,
    }.items(),
  )

  debug = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory("oakd"), "launch/debug.launch.py")]
    ),
    launch_arguments={
      "input_image_topic": namespace + "/" + input_image_topic,
      "debug_image_topic": namespace + "/yolo/" + debug_image_topic,
      "namespace": namespace,
    }.items(),
  )

  ball_location = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [
        os.path.join(
          get_package_share_directory("oakd"), "launch/balls_location.launch.py"
        )
      ]
    ),
    launch_arguments={
      "depth_image_topic": depth_image_topic,
      "pose_topic": baselink_pose_topic,
      "namespace": namespace,
      "tracking_topic": namespace + "/yolo/" + tracking_topic,
    }.items(),
  )

  goalpose = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory("oakd"), "launch/goalpose.launch.py")]
    ),
    launch_arguments={
      "pose_topic": baselink_pose_topic,
      "namespace": namespace,
      "state_n_goalpose_topic": state_n_goalpose_topic,
      "ball_goalpose_topic": ball_goalpose_topic,
    }.items(),
  )

  transforms = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory("oakd"), "launch/transforms.launch.py")]
    ),
    launch_arguments={"namespace": namespace}.items(),
  )

  republish = IncludeLaunchDescription(
    PythonLaunchDescriptionSource(
      [os.path.join(get_package_share_directory("oakd"), "launch/republish.launch.py")]
    )
  )

  dumb_process = ExecuteProcess(cmd=[["sleep 1"]], shell=True)

  return LaunchDescription(
    [
      dumb_process,
      RegisterEventHandler(
        OnProcessStart(
          target_action=dumb_process,
          on_start=[
            transforms,
          ],
        )
      ),
      RegisterEventHandler(
        OnExecutionComplete(
          target_action=transforms,
          on_completion=[
            cam_driver,
          ],
        )
      ),
      RegisterEventHandler(
        OnExecutionComplete(
          target_action=cam_driver,
          on_completion=[
            yolov8_bringup,
          ],
        )
      ),
      RegisterEventHandler(
        OnExecutionComplete(
          target_action=yolov8_bringup,
          on_completion=[
            debug,
          ],
        )
      ),
      RegisterEventHandler(
        OnExecutionComplete(
          target_action=debug,
          on_completion=[
            ball_location,
          ],
        )
      ),
      RegisterEventHandler(
        OnExecutionComplete(
          target_action=ball_location,
          on_completion=[
            goalpose,
          ],
        )
      ),
    ]
  )
