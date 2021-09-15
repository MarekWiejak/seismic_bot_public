cmake_minimum_required(VERSION 3.0.2)
project(tic_crane)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  std_msgs
  message_generation
  actionlib_msgs
  actionlib
)

## Generate actions in the 'action' folder
add_action_files(
  DIRECTORY action
  FILES MoveToPosition.action
        Recover.action
)

## Generate added messages and services with any dependencies listed here
generate_messages(
  DEPENDENCIES
  std_msgs 
  actionlib_msgs
)

catkin_package(
#  INCLUDE_DIRS
#  LIBRARIES 
 CATKIN_DEPENDS roscpp actionlib_msgs raspi_gpio
#  DEPENDS
)

include_directories(
    include
    ${catkin_INCLUDE_DIRS}
)

find_library(PololuTIC_LIBRARY pololu-tic-1 HINTS $ENV{HOME}/usr/tic/lib )   
set(PololuTIC_INCLUDE_DIRS $ENV{HOME}/usr/tic/include/libpololu-tic-1)

include_directories(
  ${PololuTIC_INCLUDE_DIRS}
)

add_library(tic_crane_lib src/motor_handle.cpp src/move_to_position_action.cpp src/recover_action.cpp)
target_link_libraries(tic_crane_lib ${catkin_LIBRARIES} ${PololuTIC_LIBRARY} )
add_dependencies(tic_crane_lib tic_crane_generate_messages_cpp)

add_executable(tic_crane_node src/tic_crane_node.cpp)
target_link_libraries(tic_crane_node tic_crane_lib)
