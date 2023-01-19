cmake_minimum_required(VERSION 2.8.3)
project(halo_radar)

add_compile_options(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
    marine_radar_control_msgs
    marine_sensor_msgs
    roscpp
)

catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
   CATKIN_DEPENDS marine_radar_control_msgs marine_sensor_msgs
#  DEPENDS system_lib
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)

set (SOURCES
    src/halo_radar/halo_radar.cpp
    src/halo_radar/halo_radar_node.cpp
)

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} ${catkin_EXPORTED_TARGETS})

target_link_libraries(${PROJECT_NAME}
    ${catkin_LIBRARIES}
)
