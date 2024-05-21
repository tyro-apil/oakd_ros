"""cam_driver.py - Camera driver for the OAK-D camera."""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from cv_bridge import CvBridge

from datetime import timedelta
import depthai as dai

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
    self.device.setIrLaserDotProjectorIntensity(1.0)

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

    # if inRgb is not None:
    #   self.get_logger().info('Received RGB frame')
    # if inDepth is not None: 
    #   self.get_logger().info('Received Depth frame')

    if inRgb is not None and inDepth is not None:
      rgb_frame = inRgb.getCvFrame()
      depth_frame = inDepth.getCvFrame()
      # Convert the frame to a ROS Image message and publish it
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
