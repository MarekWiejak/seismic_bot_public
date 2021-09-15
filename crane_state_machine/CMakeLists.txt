cmake_minimum_required(VERSION 3.0.2)
project(crane_state_machine)

## Compile as C++11, supported in ROS Kinetic and newer
# add_compile_options(-std=c++11)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  actionlib
  actionlib_msgs
  message_generation
  rospy
  std_msgs
  std_srvs
  message_runtime
)

# catkin_python_setup()

add_action_files(
  DIRECTORY action
  FILES CraneStateMachine.action
)

generate_messages(
  DEPENDENCIES
  std_msgs actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stepper_motor
 CATKIN_DEPENDS rospy std_msgs std_srvs actionlib_msgs raspi_gpio tic_crane
#  DEPENDS system_lib
)


include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
scripts/crane_state_machine_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)