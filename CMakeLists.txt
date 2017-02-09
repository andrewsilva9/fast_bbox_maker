cmake_minimum_required(VERSION 2.8.3)
project(fast_bbox_maker)

find_package(catkin REQUIRED COMPONENTS
  sensor_msgs
  cv_bridge
  std_msgs
  message_generation
)

generate_messages(
  DEPENDENCIES
  sensor_msgs
  std_msgs
)

catkin_package(
  CATKIN_DEPENDS rospy sensor_msgs cv_bridge message_runtime
)

include_directories(
  ${catkin_INCLUDE_DIRS}
)