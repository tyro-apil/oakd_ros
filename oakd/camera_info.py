"""camera_info.py - CameraInfo publisher node"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import CameraInfo
from rclpy.qos import qos_profile_sensor_data

class CameraInfoPublisher(Node):
  def __init__(self):
    super().__init__('camera_info_node')

    self.declare_parameter('height', 720)
    self.declare_parameter('width', 1080)
    self.declare_parameter('distortion_model', 'rational_polynomial')
    self.declare_parameter('d', [0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2, 0.2])
    self.declare_parameter('k', [1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0, 1.0])
    self.declare_parameter('p', [761.81488037, 0.0, 646.52478027, 0.0, 0.0, 761.15325928, 361.41662598,0.0, 0.0, 0.0, 1.0, 0.0])

    self.publisher_ = self.create_publisher(CameraInfo, 'camera_info', 10)
    timer_period = 0.05  # seconds
    self.timer = self.create_timer(timer_period, self.timer_callback)
    self.get_logger().info('CameraInfo publisher node started')

    self.camera_info = self.get_camera_info()

  def get_camera_info(self):
    camera_info_msg = CameraInfo()
    camera_info_msg.height = self.get_parameter('height').get_parameter_value().integer_value
    camera_info_msg.width = self.get_parameter('width').get_parameter_value().integer_value
    camera_info_msg.distortion_model = self.get_parameter('distortion_model').get_parameter_value().string_value
    camera_info_msg.d = self.get_parameter('d').get_parameter_value().double_array_value
    camera_info_msg.k = self.get_parameter('k').get_parameter_value().double_array_value     
    camera_info_msg.p = self.get_parameter('p').get_parameter_value().double_array_value   
    return camera_info_msg

  def timer_callback(self):
    self.publisher_.publish(self.camera_info)

def main(args=None):
  rclpy.init(args=args)
  camera_info_publisher = CameraInfoPublisher()
  rclpy.spin(camera_info_publisher)
  camera_info_publisher.destroy_node()
  rclpy.shutdown()

if __name__ == '__main__':
    main()
