# OakD-Pro W spatial detections

This package detects colored balls(red, blue, purple) and obtains their location in gamefield using the depth-map published by oakd, and then publishes goalpose for the robot.

## Requirements

1. A ***/nav2_msgs/msg/Odometry*** message to obtain **base_link** pose w.r.t. **map**.  
2. An accurate transform from **base_link** to **camera_optical_frame**.

## How to use this package

Package: ***oakd***  
A top level python launch file ***oakd.launch.py*** launches all launch files from camera_driver to goalpose_publisher.
```
ros2 launch oakd oakd.launch.py
```