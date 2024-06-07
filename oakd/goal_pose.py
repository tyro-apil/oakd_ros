"""goal_pose.py - goal_pose_node
Publishes goalPose message w.r.t. map frame
"""
from oakd_msgs.msg import SpatialBallArray

import rclpy
from rclpy.node import Node, Parameter
from rcl_interfaces.msg import SetParametersResult
from rclpy.qos import QoSProfile
from rclpy.qos import QoSReliabilityPolicy

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from math import sqrt, atan2, cos, sin
import numpy as np
from scipy.spatial.transform import Rotation as R

BALL_DIAMETER = 0.190

def get_dist(point) -> float:
  """Calculate euclidean distance between two 2D points"""
  return sqrt(point[0]**2 + point[1]**2)

class GoalPose(Node): 
  def __init__(self):
    super().__init__('goal_pose_node')
    
    qos_profile = QoSProfile(depth=10)
    qos_profile.reliability = QoSReliabilityPolicy.BEST_EFFORT

    self.create_timer(0.02, self.publish_track_state)
    # x, y, x, y
    self.declare_parameter("corner_limits", [1.65, 1.55, -1.65, -0.50])   # bottom-left to bottom-right in clock wise
    self.declare_parameter("team_color", "red")
    self.declare_parameter("goalpose_limits", [-0.5000, 0.2000, -1.5684, 1.5543])   # xmin, xmax, ymin, ymax
    self.add_on_set_parameters_callback(self.params_set_callback)

    self.goal_pose_publisher = self.create_publisher(
      PoseStamped,
      '/ball_pose_topic',
      10
    )
    self.tracking_state_pub = self.create_publisher(
      Bool,
      '/is_ball_tracked',
      10
    )
    self.baselink_pose_subscriber = self.create_subscription(
      Odometry,
      "/odometry/filtered",
      self.baselink_pose_callback,
      qos_profile=qos_profile
    )
    self.baselink_pose_subscriber  # prevent unused variable warning
    self.balls_baselink_subscriber = self.create_subscription(
      SpatialBallArray,
      "balls_map",
      self.data_received_cb,
      10
    )
    self.balls_baselink_subscriber  # prevent unused variable warning

    self.corner_limits = self.get_parameter("corner_limits").get_parameter_value().double_array_value
    self.team_color = self.get_parameter("team_color").get_parameter_value().string_value
    self.goalpose_limits = self.get_parameter("goalpose_limits").get_parameter_value().double_array_value 
    
    self.translation_map2base = None
    self.quaternion_map2base = None
    self.goalpose_map = PoseStamped()
    ### 
    self.max_xy_limit_ = [3.0, 3.0]        # Ignore detections farther than this distance w.r.t. map_frame
    ###
    self.x_offset_baselink_ = 0.60                  # Shifting to align intake with ball
    self.y_offset_baselink_ = 0.15
    
    self.tracked_id = None
    self.is_ball_tracked = Bool()
    
    self.get_logger().info(f"Goalpose node started")  

  def params_set_callback(self, params):
    success = False
    for param in params:
      if param.name == 'team_color':
        if param.type_ == Parameter.Type.STRING:
          success = True
          self.team_color = param.value
    self.get_logger().info(f"team_color: {self.team_color}")
    return SetParametersResult(successful=success)
  
  def publish_track_state(self):
    self.tracking_state_pub.publish(self.is_ball_tracked)
  
  def publish_goalPose(self):
    self.goal_pose_publisher.publish(self.goalPose_map)
    self.get_logger().info(f"\nposition: {self.goalPose_map.pose.position}\n orientation: {self.goalPose_map.pose.orientation}")

  def baselink_pose_callback(self, pose_msg: Odometry):
    self.translation_map2base = np.zeros(3)
    self.translation_map2base[0] = pose_msg.pose.pose.position.x
    self.translation_map2base[1] = pose_msg.pose.pose.position.y
    self.translation_map2base[2] = pose_msg.pose.pose.position.z

    self.quaternion_map2base = np.zeros(4)
    self.quaternion_map2base[0] = pose_msg.pose.pose.orientation.x
    self.quaternion_map2base[1] = pose_msg.pose.pose.orientation.y
    self.quaternion_map2base[2] = pose_msg.pose.pose.orientation.z
    self.quaternion_map2base[3] = pose_msg.pose.pose.orientation.w

  def data_received_cb(self, Balls_msg: SpatialBallArray):
    if self.translation_map2base is not None:
      self.get_logger().info(f"Number of deteted balls: {len(Balls_msg.spatial_balls)}")
      team_colored_balls = self.get_team_colored_balls(Balls_msg.spatial_balls)
      self.get_logger().info(f"Number of team colored balls: {len(team_colored_balls)}")
      
      if len(team_colored_balls)>0:
        self.target_ball_location = []
        
        # Assume tracked ball is lost
        prevBall_lost=True
        target_index=None
        
        # Check if tracked ball is still in view
        if self.tracked_id is not None:
          
          for i, ball in enumerate(team_colored_balls):
            if self.tracked_id == int(ball.tracker_id):
              prevBall_lost=False
              target_index=i
              break
        
        # If tracked ball is lost, track the nearest ball
        if prevBall_lost:
          target_index = self.get_min_distance_index(team_colored_balls)
          self.tracked_id = int(team_colored_balls[target_index].tracker_id)
          
        target_ball_location = [team_colored_balls[target_index].position.x , (team_colored_balls[target_index].position.y)]
        goalPose_map = self.get_goalPose_map(target_ball_location)
        self.set_goalPose(goalPose_map)
        self.publish_goalPose()
        self.is_ball_tracked.data = True   
      else:
        self.is_ball_tracked.data = False
  
  def get_team_colored_balls(self, balls):
    new_list = list(filter(lambda ball: 
      (ball.class_name == self.team_color) 
      and (ball.position.x < self.max_xy_limit_[0])  
      and (ball.position.y < self.max_xy_limit_[1])
      , balls)
    )
    return new_list
      
  def get_min_distance_index(self, balls):
    """Return index of ball at min distance from base_link inside SpatialBallArray msg"""
    min_distance_index=0
    min_distance = get_dist((balls[min_distance_index].position.x, balls[min_distance_index].position.y))
    for i, ball in enumerate(balls):
      dist = get_dist((ball.position.x, ball.position.y))
      if dist < min_distance:
        min_distance = dist
        min_distance_index = i
    return min_distance_index

  def get_goalPose_map(self, target_ball_location):
    goalpose_map = PoseStamped()
    goalpose_map.header.stamp = self.get_clock().now().to_msg()
    goalpose_map.header.frame_id = "map"
    # breakpoint()
    yaw = self.get_goalPose_yaw(target_ball_location)
    target_map = [0.0, 0.0, 0.0]
    target_map[0] = target_ball_location[0] + (-self.x_offset_baselink_ * cos(yaw) - self.y_offset_baselink_ * sin(yaw))
    target_map[1] = target_ball_location[1] + (-self.x_offset_baselink_ * sin(yaw) + self.y_offset_baselink_ * cos(yaw))
    clamped_target_map = self.clamp_target(target_map)

    goalpose_map.pose.position.x = float(clamped_target_map[0])
    goalpose_map.pose.position.y = float(clamped_target_map[1])
    goalpose_map.pose.position.z = float(clamped_target_map[2])

    q_goalpose = R.from_euler('ZYX', [yaw, 0., 0.]).as_quat()
    goalpose_map.pose.orientation.x = q_goalpose[0]
    goalpose_map.pose.orientation.y = q_goalpose[1]
    goalpose_map.pose.orientation.z = q_goalpose[2]
    goalpose_map.pose.orientation.w = q_goalpose[3]

    return goalpose_map

  def get_goalPose_yaw(self, target_ball_location):
    # return 0.0
    base2ball_vec = [target_ball_location[0]-self.translation_map2base[0], target_ball_location[1]-self.translation_map2base[1]]
    yaw= atan2(base2ball_vec[1], base2ball_vec[0])
    return yaw

  def set_goalPose(self, goalPose_map):
    self.goalPose_map = goalPose_map

  def clamp_target(self, target_map):
    clampped_target = target_map
    # if target_map[0] < self.goalpose_limits[0]:
    #   clampped_target[0] = self.goalpose_limits[0]
    # elif target_map[0] > self.goalpose_limits[1]:
    #   clampped_target[0] = self.goalpose_limits[1]

    # if target_map[1] < self.goalpose_limits[2]:
    #   clampped_target[1] = self.goalpose_limits[2]
    # elif target_map[1] > self.goalpose_limits[3]:
    #   clampped_target[1] = self.goalpose_limits[3]

    return clampped_target

  def get_x_offset(self, target_ball_location):
    return 0.00
  
  def get_y_offset(self, target_ball_location):
    return 0.00
  
def main(args=None):
  rclpy.init(args=args)

  goal_pose_node = GoalPose()

  rclpy.spin(goal_pose_node)

  # Destroy the node explicitly
  # (optional - otherwise it will be done automatically
  # when the garbage collector destroys the node object)
  goal_pose_node.destroy_node()
  rclpy.shutdown()


if __name__ == '__main__':
  main()