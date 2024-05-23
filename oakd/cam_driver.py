"""cam_driver.py - Camera driver for the OAK-D camera."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from rclpy.qos import qos_profile_sensor_data

from datetime import timedelta
import depthai as dai
import cv2
import numpy as np

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
    padded_image[top:top + new_h, left:left + new_w] = resized_image
  else:  # Color image
    padded_image[top:top + new_h, left:left + new_w, :] = resized_image

  return padded_image

class DepthAICameraHandler(Node):
  def __init__(self):
    super().__init__('camera_handler_node')
    # Camera parameters
    self.fps_ = 20
    self.rgb_width_ = 1280
    self.rgb_height_ = 720
    self.mono_resolution_ = dai.MonoCameraProperties.SensorResolution.THE_720_P
    self.alpha_ = None

    # Create a publisher for the RGB images
    self.rgb_publisher_ = self.create_publisher(Image, 'rgb/raw', 10)
    self.depth_publisher_ = self.create_publisher(Image, 'stereo/depth/raw', 10)
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
    camRgb.setPreviewSize(self.rgb_width_, self.rgb_height_)
    camRgb.Properties.previewKeepAspectRatio = True
    camRgb.setFps(self.fps_)

    left.setResolution(self.mono_resolution_)
    left.setCamera("left")
    left.setFps(self.fps_)
    right.setResolution(self.mono_resolution_)
    right.setCamera("right")
    right.setFps(self.fps_)

    stereo.setLeftRightCheck(True)
    stereo.setDepthAlign(rgbCamSocket)
    # stereo.setOutputSize(self.rgb_width_, self.rgb_height_)
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
      raise Exception('Could not read calibration data')

    self.get_logger().info('Connected cameras: {}'.format(self.device.getConnectedCameraFeatures()))
    self.get_logger().info('USB speed: {}'.format(self.device.getUsbSpeed().name))
    if self.device.getBootloaderVersion() is not None:
      self.get_logger().info('Bootloader version: {}'.format(self.device.getBootloaderVersion()))
    self.get_logger().info('Device name: {} Product name: {}'.format(self.device.getDeviceName(), self.device.getProductName()))
    self.get_logger().info('Driver node started')

    # Output queue will be used to get the frames from the output defined above
    self.qRgb = self.device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
    self.qDepth = self.device.getOutputQueue(name="depth", maxSize=4, blocking=False)

    # Create a timer to periodically call the timer_callback function
    self.timer = self.create_timer(0.03, self.timer_callback)

  def timer_callback(self):
    inRgb = self.qRgb.tryGet()  # Non-blocking call, will return None if no new data has arrived
    inDepth = self.qDepth.tryGet()  # Non-blocking call, will return None if no new data has arrived

    if inRgb is not None and inDepth is not None:
      rgb_frame = inRgb.getCvFrame()
      depth_frame = inDepth.getCvFrame()

      #Resize the opencv frames
      # rgb_frame = cv2.resize(rgb_frame, (1280, 720), interpolation=cv2.INTER_AREA)
      depth_frame = cv2.resize(depth_frame, (1280, 720), interpolation=cv2.INTER_AREA)
      # rgb_frame = paddedResize(rgb_frame, (480, 640))
      # depth_frame = paddedResize(depth_frame, (480, 640))

      # Convert the frame to a ROS Image message and publish it
      # self.get_logger().info(f'RGB frame shape {rgb_frame.shape} Depth frame shape {depth_frame.shape}')
      rgbImg_ros_msg = self.bridge.cv2_to_imgmsg(rgb_frame, encoding='bgr8')
      depthImg_ros_msg = self.bridge.cv2_to_imgmsg(depth_frame, encoding='16UC1')
      self.rgb_publisher_.publish(rgbImg_ros_msg)
      self.depth_publisher_.publish(depthImg_ros_msg)

def main(args=None):
  rclpy.init(args=args)
  node = DepthAICameraHandler()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
  main()
