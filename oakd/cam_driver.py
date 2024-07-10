"""cam_driver.py - Camera driver for the OAK-D camera."""

from datetime import timedelta

import cv2
import depthai as dai
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from std_msgs.msg import Header


def paddedResize(image, target_size, pad_color=(0, 0, 0)):
  h, w = image.shape[:2]
  target_h, target_w = target_size

  # Calculate the scaling factor to maintain aspect ratio
  scale = min(target_w / w, target_h / h)

  # Compute new dimensions of the image
  new_w = int(w * scale)
  new_h = int(h * scale)

  # Resize the image
  resized_image = cv2.resize(image, (new_w, new_h), interpolation=cv2.INTER_AREA)
  padded_image = None

  # Check if the image is grayscale or color
  if len(image.shape) == 2:  # Grayscale image
    if isinstance(pad_color, tuple):
      pad_color = pad_color[0]  # Take the first element for grayscale padding

    # Create a new image of the target size with the padding color for grayscale
    padded_image = np.full((target_h, target_w), pad_color, dtype=np.uint16)
  else:  # Color image
    # Create a new image of the target size with the padding color for color
    padded_image = np.full((target_h, target_w, 3), pad_color, dtype=np.uint8)

  # Compute the top-left corner coordinates to center the resized image
  top = (target_h - new_h) // 2
  left = (target_w - new_w) // 2

  # Place the resized image on the padded image
  if len(image.shape) == 2:  # Grayscale image
    padded_image[top : top + new_h, left : left + new_w] = resized_image
  else:  # Color image
    padded_image[top : top + new_h, left : left + new_w, :] = resized_image

  return padded_image


class DepthAICameraHandler(Node):
  def __init__(self):
    super().__init__("camera_handler_node")
    self.declare_parameter("width", 1280)
    self.declare_parameter("height", 720)
    self.declare_parameter("fps", 30)
    # Camera parameters
    self.__fps = self.get_parameter("fps").get_parameter_value().integer_value
    self.__rgb_width = self.get_parameter("width").get_parameter_value().integer_value
    self.__rgb_height = self.get_parameter("height").get_parameter_value().integer_value
    self.mono_resolution_ = dai.MonoCameraProperties.SensorResolution.THE_720_P
    self.alpha_ = None

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )

    # Create a publisher for the RGB images
    self.rgb_publisher_ = self.create_publisher(
      Image, "rgb/rect", qos_profile=image_qos_profile
    )
    self.depth_publisher_ = self.create_publisher(
      Image, "stereo/depth", qos_profile=image_qos_profile
    )
    self.bridge = CvBridge()

    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    camRgb = pipeline.create(dai.node.Camera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)
    demux = pipeline.create(dai.node.MessageDemux)

    rgbOut = pipeline.create(dai.node.XLinkOut)
    depthOut = pipeline.create(dai.node.XLinkOut)

    rgbOut.setStreamName("rgb")
    depthOut.setStreamName("depth")

    # Properties
    sync.setSyncThreshold(timedelta(milliseconds=20))

    rgbCamSocket = dai.CameraBoardSocket.CAM_A

    camRgb.setBoardSocket(rgbCamSocket)
    camRgb.setPreviewSize(self.__rgb_width, self.__rgb_height)
    camRgb.Properties.previewKeepAspectRatio = True
    camRgb.setFps(self.__fps)

    left.setResolution(self.mono_resolution_)
    left.setCamera("left")
    left.setFps(self.__fps)
    right.setResolution(self.mono_resolution_)
    right.setCamera("right")
    right.setFps(self.__fps)

    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(rgbCamSocket)
    # stereo.setOutputSize(self.__rgb_width, self.__rgb_height)
    # stereo.setOutputKeepAspectRatio(True)

    # Linking
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(sync.inputs["depth"])
    camRgb.preview.link(sync.inputs["rgb"])
    sync.out.link(demux.input)
    demux.outputs["rgb"].link(rgbOut.input)
    demux.outputs["depth"].link(depthOut.input)

    # Calibration Mesh for rectifying rgb
    camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)
    if self.alpha_ is not None:
      camRgb.setCalibrationAlpha(self.alpha_)
      stereo.setAlphaScaling(self.alpha_)

    # Connect to device and start pipeline
    self.device = dai.Device(pipeline)
    self.device.setIrLaserDotProjectorIntensity(0.5)

    # For now, RGB needs fixed focus to properly align with depth.
    # This value was used during calibration
    try:
      calibData = self.device.readCalibration2()
      lensPosition = calibData.getLensPosition(rgbCamSocket)
      if lensPosition:
        camRgb.initialControl.setManualFocus(lensPosition)
    except:
      raise Exception("Could not read calibration data")

    self.get_logger().info(
      f"Connected camera: {self.device.getConnectedCameraFeatures()}"
    )
    self.get_logger().info(f"USB speed: {self.device.getUsbSpeed().name}")
    self.get_logger().info(
      f"Device name: {self.device.getDeviceName()} Product name: {self.device.getProductName()}"
    )
    self.get_logger().info("Driver node started")

    # Output queue will be used to get the frames from the output defined above
    self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    self.qDepth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    # Create a timer to periodically call the timer_callback function
    self.timer = self.create_timer(0.05, self.timer_callback)

  def timer_callback(self):
    inRgb = (
      self.qRgb.tryGet()
    )  # Non-blocking call, will return None if no new data has arrived
    inDepth = (
      self.qDepth.tryGet()
    )  # Non-blocking call, will return None if no new data has arrived

    if inRgb is None or inDepth is None:
      return

    rgb_frame = inRgb.getCvFrame()
    depth_frame = inDepth.getCvFrame()

    msg_header = Header()
    msg_header.stamp = self.get_clock().now().to_msg()
    msg_header.frame_id = "oak_rgb_camera_link_optical"

    # Resize the opencv frames
    depth_frame = cv2.resize(depth_frame, (1280, 720), interpolation=cv2.INTER_AREA)

    # Convert the frame to a ROS Image message and publish it
    rgbImg_ros_msg = self.bridge.cv2_to_imgmsg(
      rgb_frame,
      encoding="bgr8",
      header=msg_header,
    )
    depthImg_ros_msg = self.bridge.cv2_to_imgmsg(
      depth_frame, encoding="16UC1", header=msg_header
    )
    self.rgb_publisher_.publish(rgbImg_ros_msg)
    self.depth_publisher_.publish(depthImg_ros_msg)


def main(args=None):
  rclpy.init(args=args)
  node = DepthAICameraHandler()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
