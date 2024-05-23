"""rviz_markers.py - marker_node
Publishes markerArray message of detected balls to visualise in RVIZ
"""
import rclpy
from rclpy.node import Node, Parameter
from rclpy.duration import Duration

import rclpy.serialization
from oakd_msgs.msg import SpatialBallArray
from visualization_msgs.msg import Marker, MarkerArray 

from typing import List, Tuple
from dataclasses import dataclass


from rcl_interfaces.msg import SetParametersResult

# Diameter of size 3 ball in meters
BALL_DIAMETER = 0.190 

@dataclass
class MarkerData:
  position: Tuple       ## Position of ball center (x,y,z)
  tracker_id: str       ## Tracker id assigned to ball
  class_id: int         ## Use class id to change color of balls
  class_name: str

class MarkerBroadcaster(Node):
  def __init__(self):
    super().__init__('marker_node')

    self.declare_parameter("team_color", "red")
    self.team_color = self.get_parameter("team_color").get_parameter_value().string_value

    self.add_on_set_parameters_callback(self.params_cb)
    
    self.balls_location_subscriber = self.create_subscription(
      SpatialBallArray,
      'balls_map',
      self.location_received_callback,
      10)
    self.marker_publisher = self.create_publisher(
      MarkerArray,
      'balls_detected',
      10)
    
    self.balls_location_subscriber  # prevent unused variable warning
    self.detected_markers = MarkerArray()
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
    marker = Marker()
    marker.header.frame_id = "map"

    marker.ns = "oak_detections"
    marker.id = int(ball.tracker_id)
    marker.type = Marker.SPHERE
    marker.action = Marker.ADD
    marker.frame_locked = False

    marker.pose.position.x = ball.position[0]
    marker.pose.position.y = ball.position[1]
    marker.pose.position.z = ball.position[2]

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

    marker.lifetime = Duration(seconds=0.25).to_msg()
    marker.text = ball.class_name

    return marker
  
  def location_received_callback(self, msg: SpatialBallArray):
    marker_array = MarkerArray()
    
    for ball in msg.spatial_balls:
      
      position=(ball.position.x, ball.position.y, ball.position.z)
      
      ## In rviz2, measurements are in meters
      ball_marker = MarkerData(position=tuple([coordinate for coordinate in position]), tracker_id=int(ball.tracker_id), class_id=ball.class_id, class_name=ball.class_name)
      
      marker = self.create_ball_marker(ball_marker)
      marker_array.markers.append(marker)
    
    self.detected_markers = marker_array
    self.marker_publisher.publish(self.detected_markers)

  

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