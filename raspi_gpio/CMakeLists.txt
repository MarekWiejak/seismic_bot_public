cmake_minimum_required(VERSION 3.0.2)
project(raspi_gpio)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

add_service_files(
  FILES EmptyBool.srv
)

add_action_files(
  DIRECTORY action
  FILES WaitForEdge.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES
 CATKIN_DEPENDS rospy std_msgs
#  DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
scripts/gpio_server_node.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)
