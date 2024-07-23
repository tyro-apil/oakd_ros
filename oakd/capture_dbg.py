import os
import random
import time

import cv2
import message_filters
import rclpy
from cv_bridge import CvBridge
from rcl_interfaces.msg import SetParametersResult
from rclpy.node import Node
from rclpy.parameter import Parameter
from sensor_msgs.msg import Image


class CaptureNode(Node):
  def __init__(self):
    super().__init__("capture_node")

    self.declare_parameter("enable_capture", False)

    self.raw_images_path = "/home/apil/work/robocon2024/cv/live_capture/raw"
    self.debug_images_path = "/home/apil/work/robocon2024/cv/live_capture/debug"

    rect_img_sub = message_filters.Subscriber(self, Image, "image_raw")
    debug_img_sub = message_filters.Subscriber(self, Image, "dbg_image")

    self._synchronizer = message_filters.ApproximateTimeSynchronizer(
      (rect_img_sub, debug_img_sub), 10, 0.05, True
    )
    self._synchronizer.registerCallback(self.img_received_callback)

    self.__enable_capture = (
      self.get_parameter("enable_capture").get_parameter_value().bool_value
    )
    self.bridge = CvBridge()
    self.last_captured_time = time.time()

    self.add_on_set_parameters_callback(self.on_set_parameters_callback)

    self.get_logger().info("Capture node initialized")

  def on_set_parameters_callback(self, params):
    for param in params:
      if param.name == "enable_capture" and param.type_ == Parameter.Type.BOOL:
        self.__enable_capture = param.value.bool_value
        # self.get_logger().info(
        #   f"Capture is now {'enabled' if self.__enable_capture else 'disabled'}"
        # )
    return SetParametersResult(successful=True)

  def img_received_callback(self, rect_img_msg: Image, debug_img_msg: Image):
    current_time = time.time()
    if current_time - self.last_captured_time < 1.0:
      return
    if random.random() < 0.05 or self.enable_capture:
      stamp = rect_img_msg.header.stamp
      file_name = f"{stamp.sec}_{stamp.nanosec}.jpg"

      rect_img = self.bridge.imgmsg_to_cv2(rect_img_msg, "bgr8")
      debug_img = self.bridge.imgmsg_to_cv2(debug_img_msg, "bgr8")

      # resized_raw_img = cv2.resize(
      #   rect_img, (rect_img_msg.width // 2, rect_img_msg.height // 2)
      # )
      # resized_dbg_img = cv2.resize(
      #   debug_img, (debug_img_msg.width // 2, debug_img_msg.height // 2)
      # )
      # combined_img = cv2.hconcat([resized_raw_img, resized_dbg_img])

      # Save images to disk
      rect_img_path = os.path.join(self.raw_images_path, file_name)
      debug_img_path = os.path.join(self.debug_images_path, file_name)

      # self.get_logger().info(f"Saving images to {rect_img_path} and {debug_img_path}")
      cv2.imwrite(rect_img_path, rect_img)
      cv2.imwrite(debug_img_path, debug_img)
      # cv2.imwrite(debug_img_path, combined_img)

      self.last_captured_time = current_time


def main(args=None):
  rclpy.init(args=args)
  node = CaptureNode()
  rclpy.spin(node)
  node.destroy_node()
  rclpy.shutdown()
