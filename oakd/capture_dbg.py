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
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


class CaptureNode(Node):
  def __init__(self):
    super().__init__("capture_node")

    self.declare_parameter("enable_capture", True)
    self.declare_parameter("capture_interval", 1.0)
    self.declare_parameter("sync", True)

    self.raw_images_path = "/home/apil/live_data/pit/raw"
    self.debug_images_path = "/home/apil/live_data/pit/debug"

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )

    # rect_img_sub = message_filters.Subscriber(
    #   self, Image, "image_raw", qos_profile=image_qos_profile
    # )
    # debug_img_sub = message_filters.Subscriber(
    #   self, Image, "dbg_image", qos_profile=image_qos_profile
    # )

    self.rect_img_sub = self.create_subscription(
      Image, "image_raw", self.rect_img_callback, qos_profile=image_qos_profile
    )
    self.debug_img_sub = self.create_subscription(
      Image, "dbg_image", self.debug_img_callback, qos_profile=image_qos_profile
    )

    # self._synchronizer = message_filters.ApproximateTimeSynchronizer(
    #   (rect_img_sub, debug_img_sub), 10, 1.0, True
    # )
    # self._synchronizer.registerCallback(self.img_received_callback)

    self.__enable_capture = (
      self.get_parameter("enable_capture").get_parameter_value().bool_value
    )
    self.capture_interval = (
      self.get_parameter("capture_interval").get_parameter_value().double_value
    )
    self.__sync = self.get_parameter("sync").get_parameter_value().bool_value

    self.bridge = CvBridge()
    self.last_captured_time_raw = time.time()
    self.last_captured_time_dbg = time.time()
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
    return SetParametersResult(successful=False)

  def rect_img_callback(self, msg: Image):
    current_time = time.time()
    if current_time - self.last_captured_time_raw < self.capture_interval:
      return
    if self.__sync and (
      abs(self.last_captured_time_raw - self.last_captured_time_raw)
      > self.capture_interval
    ):
      self.__enable_capture = False
    if self.__enable_capture:
      stamp = msg.header.stamp
      file_name = f"{stamp.sec}_{stamp.nanosec}.jpg"

      img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      img_path = os.path.join(self.raw_images_path, file_name)
      # self.get_logger().info(f"Rect. Img saving - {img_path}")
      cv2.imwrite(img_path, img)

      self.last_captured_time_raw = current_time

  def debug_img_callback(self, msg: Image):
    current_time = time.time()
    # self.get_logger().info("Debug image callback")
    if current_time - self.last_captured_time_dbg < self.capture_interval:
      return
    if self.__sync:
      self.__enable_capture = True
    if self.__enable_capture:
      stamp = msg.header.stamp
      file_name = f"{stamp.sec}_{stamp.nanosec}.jpg"

      img = self.bridge.imgmsg_to_cv2(msg, "bgr8")
      img_path = os.path.join(self.debug_images_path, file_name)
      # self.get_logger().info(f"Debug Img saving - {img_path}")
      cv2.imwrite(img_path, img)

      self.last_captured_time_dbg = current_time

  def img_received_callback(self, rect_img_msg: Image, debug_img_msg: Image):
    # self.get_logger().info("Both images combined callback")
    current_time = time.time()
    if current_time - self.last_captured_time < self.capture_interval:
      return
    if self.enable_capture:
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
