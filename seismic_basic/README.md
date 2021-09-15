## Package Summary
This packaege provides miscellaneous basic functionalities for the robot, like odometry and communication.

## Nodes

### odometry_node
Provides odometry information of the robot.

### controller2vel_node
Provides communication with ROS-Mobile Android app allowing for manual robot control (http://wiki.ros.org/ROS-Mobile).

## Subscribed Topics
- control_x_y (geometry_msgs/Twist): translation in x and y axes sent by ROS-Mobile app
- control_z (geometry_msgs/Twist): rotation in z axis sent by ROS-Mobile app
- measured_vel (geometry_msgs/Twist): measured velocity of the robot

## Published Topics
- cmd_vel (geometry_msgs/Twist): velocity command 
- /tf (tf/StampedTransform)

## Services
- set_odometry (seismic_basic/SetOdometry): takes x and y linear coordinates and z angular coordinates and sets current odometry position to given values. Returns no result.

## Parameters
- none


