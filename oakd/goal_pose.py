"""goal_pose.py - goal_pose_node
Publishes goalPose message w.r.t. map frame
"""
from yolov8_msgs.msg import BallArray

import rclpy
from rclpy.node import Node, Parameter
from rcl_interfaces.msg import SetParametersResult

from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool

from typing import List, Tuple
from math import sqrt, atan2, pi
import numpy as np
from scipy.spatial.transform import Rotation as R
from enum import Enum


class Quadrant(Enum):
  FIRST = 1
  SECOND = 2
  THIRD = 3
  FOURTH = 4

BALL_DIAMETER = 0.190

def get_dist(point: Tuple) -> float:
  """Calculate euclidean distance between two 2D points"""
  return sqrt(point[0]**2 + point[1]**2)

class GoalPose(Node): 
  def __init__(self):
    super().__init__('goal_pose_node')
    
    self.create_timer(0.02, self.publish_track_state)

    self.goal_pose_publisher = self.create_publisher(
      PoseStamped,
      'ball_pose_topic',
      10
    )
    self.tracking_state_pub = self.create_publisher(
      Bool,
      'is_ball_tracked',
      10
    )
    
    self.baselink_pose_subscriber = self.create_subscription(
      Odometry,
      "/freewheel/global",
      self.baselink_pose_callback,
      10
    )
    self.baselink_pose_subscriber  # prevent unused variable warning
    self.balls_baselink_subscriber = self.create_subscription(
      BallArray,
      "balls_map",
      self.data_received_cb,
      10
    )
    self.balls_baselink_subscriber  # prevent unused variable warning

    self.declare_parameter("team_color", "red")
    self.team_color = self.get_parameter("team_color").get_parameter_value().string_value

    self.declare_parameter("goalpose_limits", [-0.5000, 0.2000, -1.5684, 1.5543])   # xmin, xmax, ymin, ymax
    self.goalpose_limits = self.get_parameter("goalpose_limits").get_parameter_value().double_array_value 

    self.add_on_set_parameters_callback(self.params_set_callback)
    
    # Baselink pose w.r.t. map
    self.baselink_translation = np.ones(3, dtype=np.float32)
    self.baselink_quaternion = np.ones(4, dtype=np.float32)
    self.projection_map2base = np.hstack((R.from_euler('ZYX', self.baselink_ypr).as_matrix(), self.baselink_translation.reshape((-1,1))))
    self.projection_map2base = np.vstack((self.projection_map2base, [0.0, 0.0, 0.0, 1.0]))

    self.goalpose_map = PoseStamped()
    ### 
    self.max_xy_limit_ = [3.0, 3.0]        # Ignore detections farther than this distance w.r.t. map_frame
    ###
    self.X_OFFSET = -0.525                  # Shifting to align intake with ball
    self.Y_OFFSET = 0.175
    
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

  def baselink_pose_callback(self, msg: Odometry):
    self.baselink_translation = [msg.pose.pose.position.x, msg.pose.pose.position.y, msg.pose.pose.position.z]
    self.baselink_quaternion = [msg.pose.pose.orientation.w, msg.pose.pose.orientation.x, msg.pose.pose.orientation.y, msg.pose.pose.orientation.z]
    self.projection_map2base = np.hstack((R.from_quat(self.baselink_quaternion).as_matrix(), self.baselink_translation.reshape((-1,1))))
    self.projection_map2base = np.vstack((self.projection_map2base, [0.0, 0.0, 0.0, 1.0]))

  def data_received_cb(self, Balls_msg: BallArray):
    self.get_logger().info(f"Number of deteted balls: {len(Balls_msg.balls)}")
    team_colored_balls = self.get_team_colored_balls(Balls_msg.balls)
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
        
      target_ball_location = [team_colored_balls[target_index].center.position.x , (team_colored_balls[target_index].center.position.y)]
      goalPose_map = self.get_goalPose_map(target_ball_location)
      self.set_goalPose(goalPose_map)
      self.publish_goalPose()
      self.is_ball_tracked.data = True   
    else:
      self.is_ball_tracked.data = False
  
  def get_team_colored_balls(self, balls):
    new_list = list(filter(lambda ball: 
      (ball.class_name == self.team_color) 
      and (ball.center.position.x < self.max_xy_limit_[0])  
      and (ball.center.position.y < self.max_xy_limit_[1])
      , balls)
    )
    return new_list
      
  def get_min_distance_index(self, balls):
    """Return index of ball at min distance from base_link inside BallArray msg"""
    min_distance_index=0
    min_distance = get_dist((balls[min_distance_index].center.position.x, balls[min_distance_index].center.position.y))
    for i, ball in enumerate(balls):
      dist = get_dist((ball.center.position.x, ball.center.position.y))
      if dist < min_distance:
        min_distance = dist
        min_distance_index = i
    return min_distance_index

  def get_goalPose_map(self, target_ball_location):
    goalpose_map = PoseStamped()
    goalpose_map.header.stamp = self.get_clock().now().to_msg()
    goalpose_map.header.frame_id = "map"
    
    baselink_point = self.map2base([target_ball_location[0], target_ball_location[1], 0.0])
    target_x_baselink = baselink_point[0] + self.X_OFFSET
    target_y_baselink = baselink_point[1] + self.Y_OFFSET

    target_map = self.base2map([target_x_baselink, target_y_baselink, 0.0])
    target_map[0] += self.get_x_offset(target_ball_location)
    target_map[1] += self.get_y_offset(target_ball_location)
    clamped_target_map = self.clamp_target(target_map)

    yaw = self.get_goalPose_yaw(target_ball_location)

    goalpose_map.pose.position.x = float(clamped_target_map[0])
    goalpose_map.pose.position.y = float(clamped_target_map[1])
    goalpose_map.pose.position.z = float(clamped_target_map[2])

    q_goalpose = R.from_euler('ZYX', [yaw, 0., 0.]).as_quat()
    goalpose_map.pose.orientation.w = q_goalpose[0]
    goalpose_map.pose.orientation.x = q_goalpose[1]
    goalpose_map.pose.orientation.y = q_goalpose[2]
    goalpose_map.pose.orientation.z = q_goalpose[3]

    return goalpose_map

  def base2map(self, baselink_point):
    baselink_point.append(1.0)
    baselink_homogeneous_point = np.array(baselink_point, dtype=np.float32)
    map_point_homogeneous = np.dot(self.projection_map2base, baselink_homogeneous_point.reshape((-1,1)))

    # dehomogenize
    map_point = map_point_homogeneous / map_point_homogeneous[3]
    return map_point[:3]

  def get_goalPose_yaw(self, target_ball_location):
    # return 0.0
    yaw= atan2(target_ball_location[1]-self.baselink_translation[1], target_ball_location[0]-self.baselink_translation[0])
    orientation_vec = [target_ball_location[0]-self.baselink_translation[0], target_ball_location[1]-self.baselink_translation[1]]
    vec_quadrant = self.identify_quadrant(x=orientation_vec[0], y=orientation_vec[1])
    if vec_quadrant == Quadrant.FIRST or vec_quadrant == Quadrant.FOURTH:
      yaw = yaw
    else:
      yaw = yaw + pi
    return yaw


  def set_goalPose(self, goalPose_map):
    """Set goalPose as given goalPose"""
    self.goalPose_map = goalPose_map
    
  def identify_quadrant(self, x, y):
    if x>=0 and y>=0:
      return Quadrant.FIRST
    elif x<=0 and y>=0:
      return Quadrant.SECOND
    elif x<=0 and y<=0:
      return Quadrant.THIRD
    elif x>=0 and y<=0:
      return Quadrant.FOURTH

  def map2base(self, map_point):
    map_point.append(1.0)
    map_homogeneous_point = np.array(map_point, dtype=np.float32)
    baselink_point_homogeneous = np.dot(np.linalg.inv(self.projection_map2base), map_homogeneous_point.reshape((-1,1)))
    baselink_point = baselink_point_homogeneous / baselink_point_homogeneous[3]
    return baselink_point[:3]

  def clamp_target(self, target_map):
    clampped_target = target_map
    if target_map[0] < self.goalpose_limits[0]:
      clampped_target[0] = self.goalpose_limits[0]
    elif target_map[0] > self.goalpose_limits[1]:
      clampped_target[0] = self.goalpose_limits[1]

    if target_map[1] < self.goalpose_limits[2]:
      clampped_target[1] = self.goalpose_limits[2]
    elif target_map[1] > self.goalpose_limits[3]:
      clampped_target[1] = self.goalpose_limits[3]

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