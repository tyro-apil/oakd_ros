import numpy as np
import rclpy
from oakd_msgs.msg import SpatialBall, SpatialBallArray
from rclpy.node import Node
from scipy.spatial.transform import Rotation as R


class Cam2BaseTransform(Node):
  def __init__(self):
    super().__init__("cam2base_node")
    self.declare_parameter("base2cam_translation", [0.0, 0.0, 0.0])
    self.declare_parameter("base2cam_ypr", [0.0, 0.0, 0.0])
    self.declare_parameter("ball_diameter", 0.190)

    self.declare_parameter("extrapolate", False)
    self.declare_parameter("set_true_z", False)

    self.translation_base2cam = (
      self.get_parameter("base2cam_translation")
      .get_parameter_value()
      .double_array_value
    )
    self.ypr_base2cam = (
      self.get_parameter("base2cam_ypr").get_parameter_value().double_array_value
    )
    self.BALL_DIAMETER = (
      self.get_parameter("ball_diameter").get_parameter_value().double_value
    )
    self.__extrapolate = (
      self.get_parameter("extrapolate").get_parameter_value().bool_value
    )
    self.__set_true_z = (
      self.get_parameter("set_true_z").get_parameter_value().bool_value
    )

    self.proj_base2cam = np.eye(4)
    self.proj_base2cam[:3, 3] = self.translation_base2cam
    self.proj_base2cam[:3, :3] = R.from_euler(
      "ZYX", self.ypr_base2cam, degrees=True
    ).as_matrix()

    # breakpoint()
    self.balls_base_publisher = self.create_publisher(
      SpatialBallArray, "balls_baselink", 10
    )

    self.balls_cam_subscriber = self.create_subscription(
      SpatialBallArray, "balls_cam", self.balls_cam_callback, 10
    )
    self.balls_cam_subscriber  # prevent unused variable warning

    self.balls_base_msg = SpatialBallArray()
    self.get_logger().info("Camera2Base_link coordinate transformation node started.")

  def balls_cam_callback(self, msg: SpatialBallArray):
    # breakpoint()
    balls_base_msg = SpatialBallArray()
    balls_base_msg.pose_capture = msg.pose_capture
    balls_base_msg.header.stamp = msg.header.stamp
    balls_base_msg.header.frame_id = "base_link"

    for ball in msg.spatial_balls:
      ball_base_msg = SpatialBall()
      ball_base_msg = ball

      ball_cam_xyz = [ball.position.x, ball.position.y, ball.position.z]
      ball_baselink_xyz = self.cam2base(ball_cam_xyz)

      if self.__extrapolate:
        ball_baselink_xyz = self.extrapolate(ball_baselink_xyz)

      ball_base_msg.position.x = float(ball_baselink_xyz[0])
      ball_base_msg.position.y = float(ball_baselink_xyz[1])
      ball_base_msg.position.z = float(self.BALL_DIAMETER / 2)

      if self.__set_true_z:
        ball_base_msg.position.z = float(ball_baselink_xyz[2])

      balls_base_msg.spatial_balls.append(ball_base_msg)

    self.balls_base_msg = balls_base_msg
    self.balls_base_publisher.publish(self.balls_base_msg)

  def cam2base(self, cam_point):
    # breakpoint()
    cam_point.append(1.0)
    cam_homogeneous_point = np.array(cam_point, dtype=np.float32)
    base_homogeneous_point = np.dot(
      self.proj_base2cam, cam_homogeneous_point.reshape((-1, 1))
    )

    # dehomogenize
    base_point = base_homogeneous_point / base_homogeneous_point[3]
    return base_point[:3].ravel()

  def extrapolate(self, base_point):
    direction_ratio = {}
    direction_ratio["x"] = base_point[0] - self.translation_base2cam[0]
    direction_ratio["y"] = base_point[1] - self.translation_base2cam[1]
    direction_ratio["z"] = base_point[2] - self.translation_base2cam[2]

    x_extrapolated = ((self.BALL_DIAMETER / 2) - self.translation_base2cam[2]) * (
      direction_ratio["x"] / direction_ratio["z"]
    ) + self.translation_base2cam[0]
    y_extrapolated = ((self.BALL_DIAMETER / 2) - self.translation_base2cam[2]) * (
      direction_ratio["y"] / direction_ratio["z"]
    ) + self.translation_base2cam[1]
    return [x_extrapolated, y_extrapolated, self.BALL_DIAMETER / 2]


def main(args=None):
  rclpy.init(args=args)

  cam2base_coordinate_tf = Cam2BaseTransform()

  rclpy.spin(cam2base_coordinate_tf)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  cam2base_coordinate_tf.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
