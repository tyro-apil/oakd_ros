from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped

from math import sin, cos, pi

import rclpy
from rclpy.node import Node

from tf2_ros import TransformBroadcaster

BALL_DIAMETER=0.190


def yawpitchroll_to_quaternion(yaw, pitch, roll):
  cr = cos(roll * 0.5)
  sr = sin(roll * 0.5)
  cp = cos(pitch * 0.5)
  sp = sin(pitch * 0.5)
  cy = cos(yaw * 0.5)
  sy = sin(yaw * 0.5)

  qw = cr * cp * cy + sr * sp * sy
  qx = sr * cp * cy - cr * sp * sy
  qy = cr * sp * cy + sr * cp * sy
  qz = cr * cp * sy - sr * sp * cy

  return [qw, qx, qy, qz]


class Map2BaseFrame(Node):
  """! BaseLink to HomographyPlane Origin Transform
  
  Homography relation between Image Plane and 'radius' translated ground plane where centers of ball lie.
  Adjust values of translation and rotation of HomographyFrame with respect to base_link frame
  """
  def __init__(self):
    super().__init__('map2baselink_tf')
    
    self.baselink_pose_sub = self.create_subscription(
      PoseWithCovarianceStamped,
      "/odometry/filtered",
      self.pose_cb,
      10
    )
    
    self.baselink_pose = PoseWithCovarianceStamped()
    
    self.tf_broadcaster = TransformBroadcaster(self)
    self.timer = self.create_timer(0.1, self.broadcast_timer_callback)
    
    self.baselink_pose_sub
  
  def pose_cb(self, msg:PoseWithCovarianceStamped):
    self.baselink_pose = msg

  def broadcast_timer_callback(self):
    
    t = TransformStamped()

    t.header.stamp = self.get_clock().now().to_msg()
    t.header.frame_id = 'map'
    t.child_frame_id = 'base_link'
    
    t.transform.translation.x = self.baselink_pose.pose.pose.position.x    
    t.transform.translation.y = self.baselink_pose.pose.pose.position.y                      
    t.transform.translation.z = self.baselink_pose.pose.pose.position.z                        

    t.transform.rotation.w = self.baselink_pose.pose.pose.orientation.w
    t.transform.rotation.x = self.baselink_pose.pose.pose.orientation.x
    t.transform.rotation.y = self.baselink_pose.pose.pose.orientation.y
    t.transform.rotation.z = self.baselink_pose.pose.pose.orientation.z

    self.tf_broadcaster.sendTransform(t)


def main():
  rclpy.init()
  node = Map2BaseFrame()
  try:
    rclpy.spin(node)
  except KeyboardInterrupt:
    pass

  rclpy.shutdown()