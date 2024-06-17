"""goal_marker.py - target_marker_node
Publishes a arrow on top of target ball in rviz
"""

import rclpy
import rclpy.serialization
from geometry_msgs.msg import Point
from oakd_msgs.msg import SpatialBall
from rclpy.duration import Duration
from rclpy.node import Node
from visualization_msgs.msg import Marker


class MarkerBroadcaster(Node):
  def __init__(self):
    super().__init__("marker_node")

    self.__topic_sub = "target_ball"
    self.__frame_ref = "map"

    self.target_ball_subscriber = self.create_subscription(
      SpatialBall, self.__topic_sub, self.goal_received_callback, 10
    )
    self.marker_publisher = self.create_publisher(Marker, "target_ball", 10)

    self.target_ball_subscriber  # prevent unused variable warning
    self.target_marker = Marker()
    self.get_logger().info("Target Ball visualizer node started")

  def create_target_marker(self, target_position, tracker_id):
    marker = Marker()
    marker.header.frame_id = self.__frame_ref

    marker.ns = "oak"
    marker.id = int(tracker_id)
    marker.type = Marker.ARROW
    marker.action = Marker.ADD
    marker.frame_locked = False

    start_point = Point()
    start_point.x = target_position[0]
    start_point.y = target_position[1]
    start_point.z = 0.500

    end_point = Point()
    end_point.x = target_position[0]
    end_point.y = target_position[1]
    end_point.z = 0.240

    marker.points.append(start_point)
    marker.points.append(end_point)

    marker.scale.x = 0.05  # Arrow shaft diameter
    marker.scale.y = 0.10  # Arrow head diameter
    marker.scale.z = 0.05  # Arrow head length

    marker_rgb = {"r": 0.0, "g": 1.0, "b": 0.0, "a": 1.0}

    marker.color.r = marker_rgb["r"]
    marker.color.g = marker_rgb["g"]
    marker.color.b = marker_rgb["b"]
    marker.color.a = marker_rgb["a"]

    marker.lifetime = Duration(seconds=0.25).to_msg()
    return marker

  def goal_received_callback(self, ball: SpatialBall):
    target_position = (ball.position.x, ball.position.y, ball.position.z)

    marker = self.create_target_marker(target_position, ball.tracker_id)
    self.target_marker = marker
    self.marker_publisher.publish(self.target_marker)


def main(args=None):
  rclpy.init(args=args)

  marker_node = MarkerBroadcaster()

  rclpy.spin(marker_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  marker_node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
