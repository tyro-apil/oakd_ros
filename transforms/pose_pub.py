import rclpy
from rclpy.node import Node

from geometry_msgs.msg import PoseWithCovarianceStamped
from math import sin, cos, pi


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


class BaseDummyPosePublisher(Node):

  def __init__(self):
    super().__init__('odom_frame')
    self.pose_pub = self.create_publisher(PoseWithCovarianceStamped, '/odometry/filtered', 10)
    timer_period = 0.05  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)

  def timer_callback(self):
    baselink_pose = PoseWithCovarianceStamped()
    baselink_pose.header.stamp = self.get_clock().now().to_msg()
    baselink_pose.header.frame_id = "map"
    
    translation = [0.,0.,0.]
    ypr = [0., 0., 0.]
    
    q = yawpitchroll_to_quaternion(ypr[0], ypr[1], ypr[2])
    
    baselink_pose.pose.pose.position.x = translation[0]
    baselink_pose.pose.pose.position.y = translation[1]
    baselink_pose.pose.pose.position.z = translation[2]
    
    baselink_pose.pose.pose.orientation.w = q[0]
    baselink_pose.pose.pose.orientation.x = q[1]
    baselink_pose.pose.pose.orientation.y = q[2]
    baselink_pose.pose.pose.orientation.z = q[3]
    
    self.pose_pub.publish(baselink_pose)
    # self.get_logger().info('Publishing: "%s"' % baselink_pose.pose.pose)


def main(args=None):
    rclpy.init(args=args)

    base_pose = BaseDummyPosePublisher()

    rclpy.spin(base_pose)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    base_pose.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()