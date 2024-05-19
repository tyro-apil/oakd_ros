import rclpy
from rclpy.node import Node

from oakd_msgs.msg import SpatialBallArray, SpatialBall
import numpy as np


class Cam2BaseTransform(Node):

  def __init__(self):
    super().__init__('cam2base_node')
    ## Publisher of ball position data in real world
    self.balls_world_publisher = self.create_publisher(
      SpatialBallArray, "/balls_baselink", 10
    )

    self.balls_cam_subscriber = self.create_subscription(
      SpatialBallArray, "/balls_cam", self.balls_cam_callback, 10
    )

    self._ROTATION_MATRIX = np.array(
      [[-0.0195784,  -0.99941771,  0.02794497],
      [-0.02119683, -0.02752913, -0.99939624],
      [ 0.9995836,  -0.02015893, -0.02064551]]
    )
    self._T_VECS = np.array(
      [[-0.13130262],
      [ 0.58250643],
      [-0.11096934]]
    ) 
    self._TF_MATRIX = np.hstack((self._ROTATION_MATRIX, self._T_VECS))
    self._TF_MATRIX = np.vstack((self._TF_MATRIX, [0.0, 0.0, 0.0, 1.0]))

    # Store the most recent balls_world_message
    self.balls_world_msg = SpatialBallArray()
    self.create_timer(1, self.timer_callback)
    self.get_logger().info(f"Cam2BaseTransform node started.")


  def balls_cam_callback(self, msg: SpatialBallArray):
    """Set the balls_cam_msg to the transformed balls_world_msg"""
    balls_world_msg = SpatialBallArray()

    for ball in msg.spatial_balls:
      
      ball_world_msg = SpatialBall()
      ball_world_msg = ball

      cam_xyz = ball.position
      world_xyz = self.cam2world(cam_xyz)
      ball_world_msg.position.x = world_xyz[0]
      ball_world_msg.position.y = world_xyz[1]
      ball_world_msg.position.z = world_xyz[2]

      self.balls_world_msg.spatial_balls.append(ball_world_msg)

    self.balls_world_msg = balls_world_msg
    self.balls_world_publisher.publish(self.balls_world_msg)

  def cam2world(self, point):
    """Transform the point from camera frame to world frame"""
    point = np.array([point.x, point.y, point.z, 1])
    point = np.dot(self._TF_MATRIX, point)

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