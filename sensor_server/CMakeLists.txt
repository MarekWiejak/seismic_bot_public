cmake_minimum_required(VERSION 3.0.2)
project(sensor_server)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  message_runtime
  roscpp
  std_msgs
  actionlib_msgs
  actionlib
)

add_action_files(
  DIRECTORY action
  FILES Measurement.action
)

generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
  roscpp
)

# dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES measurement
  CATKIN_DEPENDS roscpp actionlib_msgs
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(sensor_server_lib src/measurement_action.cpp)
target_link_libraries(sensor_server_lib ${catkin_LIBRARIES})
add_dependencies(sensor_server_lib sensor_server_generate_messages_cpp)

add_executable(sensor_server_node src/sensor_server_node.cpp)
target_link_libraries(sensor_server_node sensor_server_lib ${catkin_LIBRARIES} )
add_dependencies(sensor_server_node sensor_server_generate_messages_cpp ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
