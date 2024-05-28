"""base_polygon.py - base_polygon_node
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import PolygonStamped, Point32

class BaseVisualizer(Node):
  def __init__(self):
    super().__init__('base_polygon_node')
    
    self.create_timer(0.02, self.timer_callback)
    self.polygon_publisher = self.create_publisher(
      PolygonStamped,
      'base_polygon',
      10)
    self.get_logger().info(f"Base_polygon visualizer node started")  
  
  def timer_callback(self):
    base_polygon_msg = PolygonStamped()
    base_polygon_msg.header.frame_id = "base_link"
    base_polygon_msg.header.stamp = self.get_clock().now().to_msg()
    
    point1 = Point32()
    point1.x = 0.70
    point1.y = 0.34
    point1.z = 0.0
    
    point2 = Point32()
    point2.x = -0.34
    point2.y = 0.34
    point2.z = 0.0
    
    point3 = Point32()
    point3.x = -0.34
    point3.y = -0.34
    point3.z = 0.0
    
    point4 = Point32()
    point4.x = 0.70
    point4.y = -0.34
    point4.z = 0.0

    
    base_polygon_msg.polygon.points = [point1, point2, point3, point4]
    
    self.polygon_publisher.publish(base_polygon_msg)
  

def main(args=None):
  rclpy.init(args=args)

  base_visualizer_node = BaseVisualizer()

  rclpy.spin(base_visualizer_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  base_visualizer_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()