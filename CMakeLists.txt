cmake_minimum_required(VERSION 3.0.2)
project(ros-imu-bias-correction)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  sensor_msgs
  std_msgs
)

catkin_package()

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
# include
  ${catkin_INCLUDE_DIRS}
)

add_executable(imu_calibrate_node src/imu_calibrate_node.cpp)

add_dependencies(imu_calibrate_node sensor_msgs_generate_messages_cpp)

target_link_libraries(imu_calibrate_node ${catkin_LIBRARIES}
)

