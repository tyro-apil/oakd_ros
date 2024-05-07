"""rviz_markers.py - marker_node
Publishes markerArray message of detected balls to visualise in RVIZ
"""
import rclpy
from rclpy.node import Node, Parameter
from rclpy.duration import Duration

import rclpy.serialization
from yolov8_msgs.msg import BallArray
from visualization_msgs.msg import Marker, MarkerArray 

from typing import List, Tuple
from dataclasses import dataclass


from rcl_interfaces.msg import SetParametersResult

# Diameter of size 3 ball in meters
BALL_DIAMETER = 0.190 

@dataclass
class MarkerData:
  """! MarkerData class
  
  Data for visualization markers of sphere type in rviz2
  """
  position: Tuple       ## Position of ball center (x,y)
  depth: float          ## Depth of ball in camera_coordinate_system
  tracker_id: str       ## Tracker id assigned to ball
  class_id: int         ## Use class id to change color of balls
  class_name: str
  


class MarkerBroadcaster(Node):
  """! MarkerBroadcaster class as a Node

  1. Publishes visualization markers for real world coordinates of balls for rviz2
  """
  
  def __init__(self):
    """! MarkerBroadcaster class constructor
        
    @return instance of MarkerBroadcaster class
    """
    super().__init__('marker_node')

    # List of colored balls to mark in RVIZ
    self.declare_parameter("team_color", "")
    self.team_color = self.get_parameter("team_color").get_parameter_value().string_array_value

    self.add_on_set_parameters_callback(self.params_cb)
    
    ## Publisher of ball position data in real world
    self.balls_location_subscriber = self.create_subscription(
      BallArray,
      '/balls_cam_coordinate',
      self.location_received_callback,
      10)
    ## Publisher of ball visualization markers
    self.marker_publisher = self.create_publisher(
      MarkerArray,
      '/ball_markers',
      10)
    
    self.balls_location_subscriber  # prevent unused variable warning
    self.get_logger().info(f"Marker node started")  
  
  def params_cb(self, params):
    success = False
    for param in params:
      if param.name == 'team_color':
        if param.type_ == Parameter.Type.STRING:
          success = True
          self.team_color = param.value
    self.get_logger().info(f"team_color: {self.team_color}")
    return SetParametersResult(successful=success)

  def create_ball_marker(self, ball: MarkerData):
    """Returns a Marker msg type object from a MarkerData object"""
    marker = Marker()
    marker.header.frame_id = "oak_rgb_camera_optical_frame"

    marker.ns = "balls_marker"
    marker.id = int(ball.tracker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.frame_locked = False

    marker.pose.position.x = ball.position[0]
    marker.pose.position.y = ball.position[1]
    marker.pose.position.z = ball.depth

    marker.pose.orientation.x = 0.0
    marker.pose.orientation.y = 0.0
    marker.pose.orientation.z = 0.0
    marker.pose.orientation.w = 1.0
    
    marker.scale.x = BALL_DIAMETER
    marker.scale.y = BALL_DIAMETER
    marker.scale.z = BALL_DIAMETER

    marker_rgb = {'r':0.,'g':0.,'b':0.,'a':0.8}
    match ball.class_id:
      ## Red ball rgb(252,0,0)
      case 2: 
        marker_rgb['r']=0.988
        marker_rgb['g']=0.
        marker_rgb['b']=0.
        
      ## Blue ball rgb(18,79,252)
      case 0:
        marker_rgb['r']=0.071
        marker_rgb['g']=0.310
        marker_rgb['b']=0.988
        
      ## Purple ball rgb(149,61,220)
      case 1:
        marker_rgb['r']=0.584
        marker_rgb['g']=0.239
        marker_rgb['b']=0.863
    
    if not ball.class_name is self.team_color:
      marker_rgb['a'] = 0.3
    
    marker.color.r = marker_rgb['r']
    marker.color.g = marker_rgb['g']
    marker.color.b = marker_rgb['b']
    marker.color.a = marker_rgb['a']

    marker.lifetime = Duration(seconds=0.5).to_msg()
    marker.text = ball.class_name

    return marker
  
  def location_received_callback(self, msg: BallArray):
    """Callback upon receiving location of balls"""
    marker_array = MarkerArray()
    
    for ball in msg.balls:
      
      center_m=(ball.center.position.x, ball.center.position.y)
      
      ## In rviz2, measurements are in meters
      ball_marker = MarkerData(position=tuple([center_data for center_data in center_m]), depth=ball.depth, tracker_id=int(ball.tracker_id), class_id=ball.class_id, class_name=ball.class_name)
      
      marker = self.create_ball_marker(ball_marker)
      marker_array.markers.append(marker)
    
    self.marker_publisher.publish(marker_array)

  

def main(args=None):
  rclpy.init(args=args)

  marker_node = MarkerBroadcaster()

  rclpy.spin(marker_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  marker_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()