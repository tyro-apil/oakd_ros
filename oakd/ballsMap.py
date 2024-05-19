import rclpy
from rclpy.node import Node

from yolov8_msgs.msg import BallArray, Ball
from geometry_msgs.msg import PoseWithCovarianceStamped

import numpy as np
from scipy.spatial.transform import Rotation as R

BALL_DIAMETER = 0.190

class Base2MapCoordinateTransform(Node):

  def __init__(self):
    super().__init__('cam2base_node')
    ## Publisher of ball position data in real world
    self.balls_world_publisher = self.create_publisher(
      BallArray, "/balls_map", 10
    )

    self.balls_cam_subscriber = self.create_subscription(
      BallArray, "/balls_baselink", self.balls_baselink_cb, 10
    )

    self.baselink_pose_subscriber = self.create_subscription(
      PoseWithCovarianceStamped,
      "/odometry/filtered",
      self.baselink_pose_callback,
      10
    )
    self.baselink_pose_subscriber  # prevent unused variable warning

    self._translation_map2base = np.ones(3, dtype=np.float32)
    self._quaternion_map2base = np.ones(4, dtype=np.float32)   # order: w, x, y, z

    self._orientation_map2base = R.from_quat(self._quaternion_map2base)

    self._tf_matrix_map2base = np.hstack((self._orientation_map2base.as_matrix(), self._translation_map2base.reshape((-1,1))))
    self._tf_matrix_map2base = np.vstack((self._tf_matrix_map2base, [0.0, 0.0, 0.0, 1.0]))

    self.balls_map_msg = BallArray()
    self.get_logger().info(f"Baselink2map coordinate transformation node started.")


  def balls_baselink_cb(self, msg: BallArray):
    """Set the balls_map_msg after receiving coordinates of balls w.r.t. baselink"""
    balls_map_msg = BallArray()

    for ball in msg.balls:
      
      ball_map_msg = Ball()
      ball_map_msg = ball

      ball_baselink_xyz = [ball.center.position.x, ball.center.position.y, BALL_DIAMETER/2]
      ball_map_xyz = self.base2map(ball_baselink_xyz)
      ball_map_msg.center.position.x = ball_map_xyz[0]
      ball_map_msg.center.position.y = ball_map_xyz[1]

      self.balls_map_msg.balls.append(ball_map_msg)

    self.balls_map_msg = balls_map_msg
    self.balls_world_publisher.publish(self.balls_map_msg)

  def base2map(self, point):
    """Transform the point from base_link frame to map frame"""
    point = np.array([point[0], point[1], point[2], 1])
    point = np.dot(self._tf_matrix_map2base, point)

    # dehomogenize
    point = point[:3] / point[3]
    return point

  def baselink_pose_callback(self, pose_msg: PoseWithCovarianceStamped):
    """Updates the pose of baselink w.r.t. map"""
    self._translation_map2base[0] = pose_msg.pose.pose.position.x
    self._translation_map2base[1] = pose_msg.pose.pose.position.y
    self._translation_map2base[2] = pose_msg.pose.pose.position.z

    self._quaternion_map2base[0] = pose_msg.pose.pose.orientation.w
    self._quaternion_map2base[1] = pose_msg.pose.pose.orientation.x
    self._quaternion_map2base[2] = pose_msg.pose.pose.orientation.y
    self._quaternion_map2base[3] = pose_msg.pose.pose.orientation.z

    self._orientation_map2base = R.from_quat(self._quaternion_map2base)
    self._rotation_map2base = self._orientation_map2base.as_matrix()

    self._tf_matrix_map2base = np.hstack((self._rotation_map2base, self._translation_map2base.reshape((-1,1))))
    self._tf_matrix_map2base = np.vstack((self._tf_matrix_map2base, [0.0, 0.0, 0.0, 1.0]))


def main(args=None):
  rclpy.init(args=args)

  base2map_coordinate_tf = Base2MapCoordinateTransform()

  rclpy.spin(base2map_coordinate_tf)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  base2map_coordinate_tf.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()