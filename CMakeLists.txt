cmake_minimum_required(VERSION 2.8.3)
project(map_subscriber)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tf
  std_msgs
  sensor_msgs
  nav_msgs
)

catkin_package(
)

include_directories(
  include ${catkin_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

add_executable(map_subscriber
  src/global_map.h
  src/global_map.cpp
  src/map_subscriber.cpp
)
add_dependencies(map_subscriber ${${PROJECT_NAME}_EXPORTED_TARGETS} ${catkin_EXPORTED_TARGETS})
target_link_libraries(map_subscriber
  ${catkin_LIBRARIES}
)