cmake_minimum_required(VERSION 3.0.2)
project(seismic_bringup)

find_package(catkin REQUIRED)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES stepper_motor
 CATKIN_DEPENDS rospy std_msgs std_srvs actionlib_msgs raspi_gpio tic_crane crane_state_machine seismic_basic
#  DEPENDS system_lib
)


include_directories(
# include
# ${catkin_INCLUDE_DIRS}
)