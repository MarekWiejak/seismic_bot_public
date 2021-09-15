cmake_minimum_required(VERSION 3.0.2)
project(seismic_basic)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  std_msgs
  geometry_msgs
  nav_msgs
  message_generation
  message_runtime
  tf
)

add_message_files(
   FILES
   Vector4int.msg
   Vector4float.msg
)

add_service_files(
  FILES
  SetOdometry.srv
)

 generate_messages(
   DEPENDENCIES
   std_msgs
 )

catkin_package(
#   INCLUDE_DIRS include
#   LIBRARIES ${PROJECT_NAME}
   CATKIN_DEPENDS roscpp rospy std_msgs geometry_msgs nav_msgs
#   DEPENDS system_lib
)
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(odometry_node src/odometry.cpp)
target_link_libraries(odometry_node ${catkin_LIBRARIES} )
add_dependencies(odometry_node ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})

catkin_install_python(PROGRAMS
  scripts/controller2vel_node.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)