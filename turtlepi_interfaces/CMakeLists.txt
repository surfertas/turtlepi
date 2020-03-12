cmake_minimum_required(VERSION 2.8.3)
project(turtlepi_interfaces)

find_package(catkin REQUIRED COMPONENTS
  message_generation
  geometry_msgs
  std_msgs
  move_base_msgs
  actionlib
)

## Generate services in the 'srv' folder
add_service_files(
  FILES
  GenerateTarget.srv
)

generate_messages(DEPENDENCIES
  move_base_msgs
  actionlib
  geometry_msgs
  std_msgs
)

catkin_package(CATKIN_DEPENDS
  message_runtime
  geometry_msgs
  std_msgs
  move_base_msgs
  actionlib
)

## Specify additional locations of header files
include_directories(
  ${catkin_INCLUDE_DIRS}
)
