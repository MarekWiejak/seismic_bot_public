cmake_minimum_required(VERSION 3.0.2)
project(task_determiner)

find_package(catkin REQUIRED COMPONENTS
  rospy
  std_msgs
  std_srvs
  message_generation
  message_runtime
  geometry_msgs
)

# Generate services in the 'srv' folder
add_service_files(
  FILES
  NewTask.srv
)

# Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs
  std_srvs
  geometry_msgs
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES task_determiner
 CATKIN_DEPENDS rospy std_msgs std_srvs
#  DEPENDS system_lib
)

include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

catkin_install_python(PROGRAMS
scripts/fake_task_generator.py
DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)