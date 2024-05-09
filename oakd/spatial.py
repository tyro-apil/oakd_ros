import rclpy
from rclpy.node import Node
import message_filters
from cv_bridge import CvBridge

from yolov8_msgs.msg import DetectionArray, BoundingBox2D, BallArray, Ball
from sensor_msgs.msg import Image, CompressedImage

import math
import numpy as np
from typing import List

class HostSpatialsCalc:
  def __init__(self):
    # Values
    self.THRESH_LOW = 200 # 20 cm
    self.THRESH_HIGH = 30000 # 30 m

  def setLowerThreshold(self, threshold_low):
    self.THRESH_LOW = threshold_low
  def setUpperThreshold(self, threshold_low):
    self.THRESH_HIGH = threshold_low
  def setDeltaRoi(self, delta):
    self.DELTA = delta

  def _check_input(self, roi, frame): # Check if input is ROI or point. If point, convert to ROI
    if len(roi) == 4: return roi
    if len(roi) != 2: raise ValueError("You have to pass either ROI (4 values) or point (2 values)!")
    # Limit the point so ROI won't be outside the frame
    self.DELTA = 4 # Take 4x4 depth pixels around point for depth averaging
    x = min(max(roi[0], self.DELTA), frame.shape[1] - self.DELTA)
    y = min(max(roi[1], self.DELTA), frame.shape[0] - self.DELTA)
    return (x-self.DELTA,y-self.DELTA,x+self.DELTA,y+self.DELTA)

  def _calc_angle(self, frame, offset, HFOV):
    return math.atan(math.tan(HFOV / 2.0) * offset / (frame.shape[1] / 2.0))

  # roi has to be list of ints
  def calc_spatials(self, depthFrame, roi, averaging_method=np.mean):
    roi = self._check_input(roi, depthFrame) # If point was passed, convert it to ROI
    xmin, ymin, xmax, ymax = roi

    # Calculate the average depth in the ROI.
    depthROI = depthFrame[int(ymin):int(ymax), int(xmin):int(xmax)]
    inRange = (self.THRESH_LOW <= depthROI) & (depthROI <= self.THRESH_HIGH)

    # Required information for calculating spatial coordinates on the host
    # HFOV = 1.254194             # OAKD PRO
    HFOV = 2.2165681500327987   # OAKD PRO-W
    averageDepth = averaging_method(depthROI[inRange])

    centroid = { # Get centroid of the ROI
      'x': int((xmax + xmin) / 2),
      'y': int((ymax + ymin) / 2)
    }

    midW = int(depthFrame.shape[1] / 2) # middle of the depth img width
    midH = int(depthFrame.shape[0] / 2) # middle of the depth img height
    bb_x_pos = centroid['x'] - midW
    bb_y_pos = centroid['y'] - midH

    angle_x = self._calc_angle(depthFrame, bb_x_pos, HFOV)
    angle_y = self._calc_angle(depthFrame, bb_y_pos, HFOV)

    spatials = {
      'z': averageDepth / 1000,
      'x': averageDepth * math.tan(angle_x) / 1000,
      'y': -averageDepth * math.tan(angle_y) / 1000
    }
    return spatials

def xywh2xyxy(xywh: List)->List:
  """Converts bbox xywh format into xyxy format"""
  xyxy = []
  xyxy.append(xywh[0]-int(xywh[2]/2))
  xyxy.append(xywh[1]-int(xywh[3]/2))
  xyxy.append(xywh[0]+int(xywh[2]/2))
  xyxy.append(xywh[1]+int(xywh[3]/2))
  return xyxy


class SpatialCalculator(Node):

  def __init__(self):
    super().__init__('spatial_node')
    ## Publisher of ball position data in real world
    self.balls_location_publisher = self.create_publisher(
        BallArray, "/balls_cam_coordinate", 10
    )

    raw_img_sub = message_filters.Subscriber(self, Image, "image_raw", qos_profile=10)  #subscriber to raw image message
    detections_sub = message_filters.Subscriber(self, DetectionArray, "tracking", qos_profile=10)  #subscriber to detections message
    
    # synchronise callback of two independent subscriptions
    self._synchronizer = message_filters.ApproximateTimeSynchronizer((raw_img_sub, detections_sub), 10, 0.5, True)
    self._synchronizer.registerCallback(self.detections_cb)

    self.bridge = CvBridge()
    self.balls_world_msg = BallArray()
    self.hostSpatials = HostSpatialsCalc()

    self.get_logger().info(f"SpatialCalculator node started.")

  def detections_cb(self, depthImg_msg: Image, detections_msg: DetectionArray):
    # Reset old data
    self.balls_world_msg = BallArray()

    depthFrame = self.bridge.imgmsg_to_cv2(depthImg_msg)
    detections = detections_msg.detections

    for detection in detections:
      # Parse bbox  
      bbox_xywh = self.parse_bbox(detection.bbox)
      # bbox_xyxy = xywh2xyxy(bbox_xywh)
      # bbox_xyxy = [int(index) for index in bbox_xyxy]

      # Calc Spatial Location
      spatials = self.hostSpatials.calc_spatials(depthFrame, bbox_xywh[:2])

      # Get 'Ball' type message for individual detection
      ball_msg = Ball()
      ball_msg.center.position.x = float(spatials['x'])
      ball_msg.center.position.y = float(spatials['y'])
      ball_msg.depth = float(spatials['z'])
      ball_msg.radius = float((bbox_xywh[2]+bbox_xywh[3])/4)  # Radius in image
      ball_msg.tracker_id = detection.id
      ball_msg.class_id = detection.class_id
      ball_msg.class_name = detection.class_name
      ball_msg.score = detection.score

      self.balls_world_msg.balls.append(ball_msg)
 
    # Publish the 'BallArray' message
    self.balls_location_publisher.publish(self.balls_world_msg)

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


if __name__ == '__main__':
  main()