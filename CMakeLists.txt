cmake_minimum_required(VERSION 2.8.3)
project(halo_radar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  marine_msgs
  rospy
)

catkin_python_setup()

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES range_and_bearing
   CATKIN_DEPENDS marine_msgs rospy
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

