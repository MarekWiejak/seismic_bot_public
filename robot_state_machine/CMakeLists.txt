cmake_minimum_required(VERSION 3.0.2)
project(robot_state_machine)

find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  message_runtime
  rospy
  std_msgs
  std_srvs
  geometry_msgs
  move_base_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stepper_motor
 CATKIN_DEPENDS rospy std_msgs std_srvs actionlib_msgs geometry_msgs move_base_msgs actionlib
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
scripts/robot_state_machine_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)