import os
from glob import glob

from setuptools import find_packages, setup

package_name = "oakd"

setup(
  name=package_name,
  version="0.0.0",
  packages=find_packages(exclude=["test"]),
  data_files=[
    ("share/ament_index/resource_index/packages", ["resource/" + package_name]),
    ("share/" + package_name, ["package.xml"]),
    (
      os.path.join("share", package_name, "launch"),
      glob(os.path.join("launch", "*launch.[pxy][yma]*")),
    ),
    (
      os.path.join("share", package_name, "config"),
      glob(os.path.join("config", "*.yaml")),
    ),
    (os.path.join("share", "yolov8_msgs", "msg"), glob(os.path.join("msg", "*.msg"))),
    (os.path.join("share", "oakd_msgs", "msg"), glob(os.path.join("msg", "*.msg"))),
  ],
  install_requires=["setuptools"],
  zip_safe=True,
  maintainer="apil",
  maintainer_email="078bct017.apil@pcampus.edu.np",
  description="TODO: Package description",
  license="TODO: License declaration",
  tests_require=["pytest"],
  entry_points={
    "console_scripts": [
      "camera_info_node = oakd.camera_info:main",
      "driver_node = oakd.cam_driver:main",
      "spatial_node = oakd.spatial:main",
      "markers_node = oakd.rviz_markers:main",
      "cam2base_node = oakd.ballsBase:main",
      "base2map_node = oakd.ballsMap:main",
      "goalpose_node = oakd.goal_pose:main",
      "cam_optical2cam_ros_tf = tfs.cam_optical2cam_ros:main",
      "base2cam_optical_tf = tfs.base2cam_optical:main",
      "target_marker_node = oakd.target_marker:main",
      "capture_node = oakd.capture_dbg:main",
    ],
  },
)
