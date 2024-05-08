import rclpy
from rclpy.node import Node

from yolov8_msgs.msg import BallArray, Ball
import numpy as np


class Cam2BaseTransform(Node):

  def __init__(self):
    super().__init__('cam2base_node')
    ## Publisher of ball position data in real world
    self.balls_world_publisher = self.create_publisher(
        BallArray, "/balls_world_coordinate", 10
    )

    self.balls_cam_subscriber = self.create_subscription(
      BallArray, "/balls_cam_coordinate", self.balls_cam_callback, 10
    )

    self._ROTATION_MATRIX = np.array([[0, 1, 0], [1, 0, 0], [0, 0, -1]])
    self._T_VECS = np.array([0, 0, 0]) 
    self._TF_MATRIX = np.hstack((self._ROTATION_MATRIX, self._T_VECS))
    self._TF_MATRIX = np.vstack((self._TF_MATRIX, [0, 0, 0, 1]))
    self._INV_TF_MATRIX = np.linalg.inv(self._TF_MATRIX)

    self.balls_world_msg = BallArray()
    self.create_timer(1, self.timer_callback)
    self.get_logger().info(f"Cam2BaseTransform node started.")


  def balls_cam_callback(self, msg: BallArray):
    """Set the balls_cam_msg to the transformed balls_world_msg"""
    self.balls_world_msg = BallArray()
    for ball in msg.balls:
      
      ball_world_msg = Ball()
      ball_world_msg = ball

      cam_xyz = ball.center.position
      world_xyz = self.cam2world(cam_xyz)
      ball_world_msg.center.position.x = world_xyz[0]
      ball_world_msg.center.position.y = world_xyz[1]
      ball_world_msg.center.position.z = world_xyz[2]

      self.balls_world_msg.balls.append(ball_world_msg)
    
    self.balls_world_publisher.publish(self.balls_world_msg)

  def cam2world(self, point):
    """cam2world the point from camera frame to world frame"""
    point = np.array([point.x, point.y, point.z, 1])
    point = np.dot(self._INV_TF_MATRIX, point)

    # dehomogenize
    point = point[:3] / point[3]
    return point

def main(args=None):
  rclpy.init(args=args)

  cam2base_node = Cam2BaseTransform()

  rclpy.spin(cam2base_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  cam2base_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()