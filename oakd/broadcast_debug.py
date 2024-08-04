import socket
import struct
import threading
import time

import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import (
  QoSDurabilityPolicy,
  QoSHistoryPolicy,
  QoSProfile,
  QoSReliabilityPolicy,
)
from sensor_msgs.msg import Image


class ImagePublisher(Node):
  def __init__(self):
    super().__init__("debug_broadcaster")

    image_qos_profile = QoSProfile(
      reliability=QoSReliabilityPolicy.BEST_EFFORT,
      history=QoSHistoryPolicy.KEEP_LAST,
      durability=QoSDurabilityPolicy.VOLATILE,
      depth=1,
    )
    self.subscription = self.create_subscription(
      Image, "dbg_image", self.listener_callback, qos_profile=image_qos_profile
    )
    self.subscription  # prevent unused variable warning
    self.bridge = CvBridge()

    # TCP socket setup
    self.tcp_ip = "0.0.0.0"  # Listen on all interfaces
    self.tcp_port = 9898  # Use any port number you like
    self.sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    self.sock.bind((self.tcp_ip, self.tcp_port))
    self.sock.listen(5)
    self.sock_clients = []

    # Start a thread to accept clients
    self.accept_thread = threading.Thread(target=self.accept_clients)
    self.accept_thread.start()

  def accept_clients(self):
    while True:
      conn, addr = self.sock.accept()
      self.get_logger().info(f"Accepted connection from {addr}")
      self.sock_clients.append(conn)

  def listener_callback(self, msg):
    try:
      # Convert ROS Image message to OpenCV image
      cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")

      # Serialize frame
      _, data = cv2.imencode(".jpg", cv_image)
      data = data.tobytes()
      size = len(data)

      # Send the size of the data first, then the data itself to all clients
      for client in self.sock_clients:
        try:
          client.sendall(struct.pack(">L", size) + data)
        except socket.error as e:
          self.get_logger().error(f"Error sending data to client: {e}")
          self.sock_clients.remove(client)
          time.sleep(1)
    except socket.error as e:
      print(f"Socket error: {e}")
      time.sleep(1)


def main(args=None):
  rclpy.init(args=args)
  image_publisher = ImagePublisher()
  rclpy.spin(image_publisher)
  for client in image_publisher.clients:
    client.close()
  image_publisher.sock.close()
  image_publisher.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
