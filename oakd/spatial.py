import math
from typing import Callable, List

import message_filters
import numpy as np
import rclpy
from cv_bridge import CvBridge
from oakd_msgs.msg import SpatialBall, SpatialBallArray
from rclpy.node import Node
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image
from yolov8_msgs.msg import BoundingBox2D, DetectionArray


class HostSpatialsCalc:
  def __init__(self, neighbourhood_pixels: int = 4) -> None:
    # Values
    self.THRESH_LOW = 200  # 20 cm
    self.THRESH_HIGH = 30000  # 30 m
    self.DELTA = neighbourhood_pixels

  def setLowerThreshold(self, threshold_low: int) -> None:
    self.THRESH_LOW = threshold_low

  def setUpperThreshold(self, threshold_high: int) -> None:
    self.THRESH_HIGH = threshold_high

  def setDeltaRoi(self, delta: int) -> None:
    self.DELTA = delta

  def _check_input(
    self, roi, frame
  ):  # Check if input is ROI or point. If point, convert to ROI
    if len(roi) == 4:
      return roi
    if len(roi) != 2:
      raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
    # Limit the point so ROI won't be outside the frame
    x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
    y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
    return (x - self.DELTA, y - self.DELTA, x + self.DELTA, y + self.DELTA)

  def _calc_angle(self, frame, offset, HFOV):
    return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

  # roi has to be list of ints
  def calc_spatials(self, depthFrame, roi, averaging_method: Callable = np.mean):
    roi = self._check_input(roi, depthFrame)  # If point was passed, convert it to ROI
    xmin, ymin, xmax, ymax = roi

    # Calculate the average depth in the ROI.
    depthROI = depthFrame[int(ymin) : int(ymax), int(xmin) : int(xmax)]
    inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)

    # Required information for calculating spatial coordinates on the host
    # HFOV = 1.25419360             # OAKD PRO : in radians
    HFOV = 2.216568150  # OAKD PRO-W : in radians
    averageDepth = averaging_method(depthROI[inRange])

    centroid = {  # Get centroid of the ROI
      "x": int((xmax + xmin) / 2),
      "y": int((ymax + ymin) / 2),
    }

    midW = int(depthFrame.shape[1] / 2)  # middle of the depth img width
    midH = int(depthFrame.shape[0] / 2)  # middle of the depth img height
    bb_x_pos = centroid["x"] - midW
    bb_y_pos = centroid["y"] - midH

    angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
    angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)

    spatials = {
      "z": averageDepth / 1000,
      "x": averageDepth * math.tan(angle_x) / 1000,
      "y": -averageDepth * math.tan(angle_y) / 1000,
    }
    return spatials


def xywh2xyxy(xywh: List) -> List:
  """Converts bbox xywh format into xyxy format"""
  xyxy = []
  xyxy.append(xywh[0] - int(xywh[2] / 2))
  xyxy.append(xywh[1] - int(xywh[3] / 2))
  xyxy.append(xywh[0] + int(xywh[2] / 2))
  xyxy.append(xywh[1] + int(xywh[3] / 2))
  return xyxy


class CameraInfoManager:
  """Camera info class to calculate spatial location of the object in the real world"""

  def __init__(self, fx, fy, cx, cy):
    self.fx = fx
    self.fy = fy
    self.cx = cx
    self.cy = cy

  def get_spatial_location(self, depth, x_img, y_img):
    spatials = {}
    x_cam = (x_img - self.cx) * depth / self.fx
    y_cam = (y_img - self.cy) * depth / self.fy
    spatials["x"] = x_cam / 1000
    spatials["y"] = y_cam / 1000
    spatials["z"] = depth / 1000
    return spatials

  def set_camera_info(self, fx, fy, cx, cy):
    self.fx = fx
    self.fy = fy
    self.cx = cx
    self.cy = cy


class SpatialCalculator(Node):
  def __init__(self):
    super().__init__("spatial_node")

    # Declare parameters
    self.declare_parameter("k", [0.0] * 9)
    self.declare_parameter("coordinate_calc_method", "SinglePointDepth")
    self.declare_parameter("host_spatials_method", "bbox_center")
    self.declare_parameter("neighbourhood_pixels", 4)
    self.declare_parameter("decimal_accuracy", 3)

    ## Publisher of ball position data in real world
    self.balls_location_publisher = self.create_publisher(
      SpatialBallArray, "balls_cam", 10
    )

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )

    raw_img_sub = message_filters.Subscriber(
      self, Image, "stereo/depth", qos_profile=image_qos_profile
    )  # subscriber to raw depth image message
    detections_sub = message_filters.Subscriber(
      self, DetectionArray, "yolo/tracking", qos_profile=10
    )  # subscriber to detections message

    self.__decimal_accuracy = (
      self.get_parameter("decimal_accuracy").get_parameter_value().integer_value
    )
    intrinsic_matrix_flat = (
      self.get_parameter("k").get_parameter_value().double_array_value
    )
    self.__coordinate_calc_method = (
      self.get_parameter("coordinate_calc_method").get_parameter_value().string_value
    )
    self.__host_spatials_method = (
      self.get_parameter("host_spatials_method").get_parameter_value().string_value
    )
    neighbourhood_pixels = (
      self.get_parameter("neighbourhood_pixels").get_parameter_value().integer_value
    )

    self.camera_info_handler = CameraInfoManager(
      intrinsic_matrix_flat[0],
      intrinsic_matrix_flat[4],
      intrinsic_matrix_flat[2],
      intrinsic_matrix_flat[5],
    )

    # synchronise callback of two independent subscriptions
    self._synchronizer = message_filters.ApproximateTimeSynchronizer(
      (raw_img_sub, detections_sub), 10, 0.05, True
    )
    self._synchronizer.registerCallback(self.detections_cb)

    self.bridge = CvBridge()
    self.balls_cam_msg = SpatialBallArray()
    self.hostSpatials = HostSpatialsCalc(neighbourhood_pixels)

    self.get_logger().info("SpatialCalculator node started.")

  def detections_cb(self, depthImg_msg: Image, detections_msg: DetectionArray):
    # Reset old data
    balls_cam_msg = SpatialBallArray()
    balls_cam_msg.header.stamp = detections_msg.header.stamp
    balls_cam_msg.header.frame_id = "oak_rgb_camera_link_optical"

    depthFrame = self.bridge.imgmsg_to_cv2(depthImg_msg)

    detections = detections_msg.detections

    for detection in detections:
      # Parse bbox
      bbox_xywh = self.parse_bbox(detection.bbox)
      center_xy = bbox_xywh[:2]
      center_xy = [int(item) for item in center_xy]

      match self.__coordinate_calc_method:
        case "SinglePointDepth":
          spatials = self.camera_info_handler.get_spatial_location(
            depthFrame[center_xy[1], center_xy[0]], center_xy[0], center_xy[1]
          )

        case "HostSpatials":
          if self.__host_spatials_method == "bbox_center":
            spatials = self.hostSpatials.calc_spatials(depthFrame, center_xy)
          elif self.__host_spatials_method == "bbox_roi":
            bbox_xyxy = xywh2xyxy(bbox_xywh)
            spatials = self.hostSpatials.calc_spatials(depthFrame, bbox_xyxy)
          else:
            raise ValueError("Invalid host spatials calculation method")

        case _:
          raise ValueError("Invalid coordinate calculation method")

      # Get 'Ball' type message for individual detection
      ball_msg = SpatialBall()
      ball_msg.position.x = round(float(spatials["x"]), self.__decimal_accuracy)
      ball_msg.position.y = round(float(spatials["y"]), self.__decimal_accuracy)
      ball_msg.position.z = round(float(spatials["z"]), self.__decimal_accuracy)

      ball_msg.tracker_id = detection.id
      ball_msg.class_id = detection.class_id
      ball_msg.class_name = detection.class_name
      ball_msg.score = detection.score

      balls_cam_msg.spatial_balls.append(ball_msg)

    self.balls_cam_msg = balls_cam_msg
    # Publish the 'BallArray' message
    self.balls_location_publisher.publish(self.balls_cam_msg)

  def parse_bbox(self, bbox_xywh: BoundingBox2D):
    """! Parse bbox from BoundingBox2D msg
    @param bbox_xywh a BoundingBox2D msg in format xywh
    @return a tuple of center_x, center_y, width, height
    """
    center_x = int(bbox_xywh.center.position.x)
    center_y = int(bbox_xywh.center.position.y)
    width = int(bbox_xywh.size.x)
    height = int(bbox_xywh.size.y)
    return [center_x, center_y, width, height]


def main(args=None):
  rclpy.init(args=args)

  spatial_node = SpatialCalculator()

  rclpy.spin(spatial_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  spatial_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
