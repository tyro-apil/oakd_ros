import rclpy
from rclpy.node import Node

from oakd_msgs.msg import SpatialBall, SpatialBallArray

import numpy as np

BALL_DIAMETER = 0.190

class Cam2BaseTransform(Node):

  def __init__(self):
    super().__init__('cam2base_node')

    self.declare_parameter("p_base2cam", [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
    p_base2cam_flat = self.get_parameter("p_base2cam").get_parameter_value().double_array_value
    self.p_base2cam_ = np.array(p_base2cam_flat, dtype=np.float32).reshape((3,4))

    self.balls_base_publisher = self.create_publisher(
      SpatialBallArray, "/balls_baselink", 10
    )

    self.balls_cam_subscriber = self.create_subscription(
      SpatialBallArray, "/balls_cam", self.balls_cam_callback, 10
    )
    self.balls_cam_subscriber # prevent unused variable warning

    self.balls_base_msg = SpatialBallArray()
    self.get_logger().info(f"Camera2Base_link coordinate transformation node started.")


  def balls_cam_callback(self, msg: SpatialBallArray):
    balls_base_msg = SpatialBallArray()
  
    for ball in msg.balls:
      
      ball_base_msg = SpatialBall()
      ball_base_msg = ball

      ball_cam_xyz = [ball.position.x, ball.position.y, ball.position.z]
      ball_baselink_xyz = self.cam2base(ball_cam_xyz)
      ball_base_msg.position.x = float(ball_baselink_xyz[0])
      ball_base_msg.position.y = float(ball_baselink_xyz[1])
      ball_base_msg.position.z = float(ball_baselink_xyz[2])

      balls_base_msg.balls.append(ball_base_msg)

    self.balls_base_msg = balls_base_msg
    self.balls_base_publisher.publish(self.balls_base_msg)

  def cam2base(self, cam_point):
    cam_point.append(1.0)
    cam_homogeneous_point = np.array(cam_point, dtype=np.float32)
    base_homogeneous_point = np.dot(self.p_base2cam_, cam_homogeneous_point.reshape((-1,1)))

    # dehomogenize
    base_point = base_homogeneous_point / base_homogeneous_point[3]
    return base_point[:3]

def main(args=None):
  rclpy.init(args=args)

  cam2base_coordinate_tf = Cam2BaseTransform()

  rclpy.spin(cam2base_coordinate_tf)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  cam2base_coordinate_tf.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()