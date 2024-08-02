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

    self.declare_params()
    self.get_params()

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

    pipeline = self.create_pipeline()

    # Connect to device and start pipeline
    self.device = dai.Device(pipeline)
    self.device.setIrLaserDotProjectorIntensity(1.0)

    controlQueue = self.device.getInputQueue("control")
    ctrl = self.create_camera_control()
    controlQueue.send(ctrl)

    # For now, RGB needs fixed focus to properly align with depth.
    # This value was used during calibration
    try:
      calibData = self.device.readCalibration2()
      lensPosition = calibData.getLensPosition(self.rgb_cam_socket)
      if lensPosition:
        self.camRgb.initialControl.setManualFocus(lensPosition)
    except:
      raise Exception("Could not read calibration data")

    self.display_log()

    # Output queue will be used to get the frames from the output defined above
    self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    self.qDepth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

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

    rgb_frame = cv2.rotate(rgb_frame, cv2.ROTATE_180)
    depth_frame = cv2.rotate(depth_frame, cv2.ROTATE_180)

    msg_header = Header()
    msg_header.stamp = self.get_clock().now().to_msg()
    msg_header.frame_id = "oak_rgb_camera_link_optical"

    # Resize the opencv frames
    depth_frame = cv2.resize(
      depth_frame, (self.__rgb_width, self.__rgb_height), interpolation=cv2.INTER_LINEAR
    )

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

  def declare_params(self):
    self.declare_parameter("width", 1280)
    self.declare_parameter("height", 720)
    self.declare_parameter("fps", 30)
    self.declare_parameter("set_alpha", False)
    self.declare_parameter("alpha", 1.0)

    self.declare_parameter("auto_exp", False)
    self.declare_parameter("auto_wb", False)
    self.declare_parameter("exp_time", 10000)
    self.declare_parameter("wb", 4000)
    self.declare_parameter("iso", 800)
    self.declare_parameter("brightness", 0)
    self.declare_parameter("contrast", 0)
    self.declare_parameter("saturation", 0)
    self.declare_parameter("sharpness", 0)
    self.declare_parameter("luma_denoise", 1)
    self.declare_parameter("chroma_denoise", 1)

    self.declare_parameter("auto_focus", True)
    self.declare_parameter("lensPos", 50)

  def get_params(self):
    self.__fps = self.get_parameter("fps").get_parameter_value().integer_value
    self.__rgb_width = self.get_parameter("width").get_parameter_value().integer_value
    self.__rgb_height = self.get_parameter("height").get_parameter_value().integer_value
    self.__set_alpha = self.get_parameter("set_alpha").get_parameter_value().bool_value
    self.__alpha = self.get_parameter("alpha").get_parameter_value().double_value

    self.auto_focus = self.get_parameter("auto_focus").get_parameter_value().bool_value
    self.lensPos = self.get_parameter("lensPos").get_parameter_value().integer_value
    self.auto_exp = self.get_parameter("auto_exp").get_parameter_value().bool_value
    self.auto_wb = self.get_parameter("auto_wb").get_parameter_value().bool_value
    self.exp_time = self.get_parameter("exp_time").get_parameter_value().integer_value
    self.wb = self.get_parameter("wb").get_parameter_value().integer_value
    self.iso = self.get_parameter("iso").get_parameter_value().integer_value
    self.brightness = (
      self.get_parameter("brightness").get_parameter_value().integer_value
    )
    self.contrast = self.get_parameter("contrast").get_parameter_value().integer_value
    self.saturation = (
      self.get_parameter("saturation").get_parameter_value().integer_value
    )
    self.sharpness = self.get_parameter("sharpness").get_parameter_value().integer_value
    self.luma_denoise = (
      self.get_parameter("luma_denoise").get_parameter_value().integer_value
    )
    self.chroma_denoise = (
      self.get_parameter("chroma_denoise").get_parameter_value().integer_value
    )

  def create_pipeline(self):
    # Create pipeline
    pipeline = dai.Pipeline()

    # Define source and output
    self.camRgb = pipeline.create(dai.node.Camera)
    left = pipeline.create(dai.node.MonoCamera)
    right = pipeline.create(dai.node.MonoCamera)
    stereo = pipeline.create(dai.node.StereoDepth)
    sync = pipeline.create(dai.node.Sync)
    demux = pipeline.create(dai.node.MessageDemux)

    rgbOut = pipeline.create(dai.node.XLinkOut)
    depthOut = pipeline.create(dai.node.XLinkOut)
    controlIn = pipeline.create(dai.node.XLinkIn)

    rgbOut.setStreamName("rgb")
    depthOut.setStreamName("depth")
    controlIn.setStreamName("control")

    # Properties
    sync.setSyncThreshold(timedelta(milliseconds=20))

    self.rgb_cam_socket = dai.CameraBoardSocket.CAM_A

    # self.camRgb.Properties.sen
    self.camRgb.setBoardSocket(self.rgb_cam_socket)
    self.camRgb.setPreviewSize(self.__rgb_width, self.__rgb_height)
    self.camRgb.Properties.previewKeepAspectRatio = True
    self.camRgb.setFps(self.__fps)

    self.mono_resolution_ = dai.MonoCameraProperties.SensorResolution.THE_720_P
    left.setResolution(self.mono_resolution_)
    left.setCamera("left")
    left.setFps(self.__fps)
    right.setResolution(self.mono_resolution_)
    right.setCamera("right")
    right.setFps(self.__fps)

    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(self.rgb_cam_socket)
    # stereo.setOutputSize(self.__rgb_width, self.__rgb_height)
    # stereo.setOutputKeepAspectRatio(True)

    # Linking
    controlIn.out.link(self.camRgb.inputControl)
    left.out.link(stereo.left)
    right.out.link(stereo.right)
    stereo.depth.link(sync.inputs["depth"])
    self.camRgb.preview.link(sync.inputs["rgb"])
    sync.out.link(demux.input)
    demux.outputs["rgb"].link(rgbOut.input)
    demux.outputs["depth"].link(depthOut.input)

    # Calibration Mesh for rectifying rgb
    self.camRgb.setMeshSource(dai.CameraProperties.WarpMeshSource.CALIBRATION)
    if self.__set_alpha:
      self.camRgb.setCalibrationAlpha(self.__alpha)
      stereo.setAlphaScaling(self.__alpha)

    return pipeline

  def create_camera_control(self):
    ctrl = dai.CameraControl()

    if self.auto_focus:
      ctrl.setAutoFocusMode(dai.CameraControl.AutoFocusMode.CONTINUOUS_VIDEO)
    else:
      ctrl.setManualFocus(self.lensPos)

    if self.auto_exp:
      ctrl.setAutoExposureEnable()
    else:
      ctrl.setManualExposure(self.exp_time, self.iso)

    if self.auto_wb:
      ctrl.setAutoWhiteBalanceMode(dai.CameraControl.AutoWhiteBalanceMode.AUTO)
    else:
      ctrl.setManualWhiteBalance(self.wb)

    ctrl.setBrightness(self.brightness)
    ctrl.setContrast(self.contrast)
    ctrl.setSaturation(self.saturation)
    ctrl.setSharpness(self.sharpness)

    ctrl.setLumaDenoise(self.luma_denoise)
    ctrl.setChromaDenoise(self.chroma_denoise)

    return ctrl

  def display_log(self):
    self.get_logger().info(
      f"Connected camera: {self.device.getConnectedCameraFeatures()}"
    )
    self.get_logger().info(f"USB speed: {self.device.getUsbSpeed().name}")
    self.get_logger().info(
      f"Device name: {self.device.getDeviceName()} Product name: {self.device.getProductName()}"
    )
    self.get_logger().info("Driver node started")


def main(args=None):
  rclpy.init(args=args)
  node = DepthAICameraHandler()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
