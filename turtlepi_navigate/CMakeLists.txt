cmake_minimum_required(VERSION 2.8.3)
project(turtlepi_navigate)

add_definitions(-std=c++11)

find_package(catkin REQUIRED COMPONENTS
  costmap_2d
  actionlib
  move_base_msgs
  geometry_msgs
  nav_msgs
  map_msgs
  std_msgs
  visualization_msgs
  tf2_geometry_msgs
  tf
  roscpp
  turtlepi_interfaces
)

find_package(Boost REQUIRED COMPONENTS system)


catkin_package(
  INCLUDE_DIRS
    include
  CATKIN_DEPENDS
    costmap_2d
    actionlib
    move_base_msgs
    nav_msgs
    map_msgs
    std_msgs
    geometry_msgs
    visualization_msgs
    tf2_geometry_msgs
    tf
    roscpp
    turtlepi_interfaces
)


###########
## Build ##
###########

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
  ${Boost_INCLUDE_DIRS}
)

add_executable(turtlepi_nav_node
  src/turtlepi_nav_node.cpp
  src/turtlepi_nav.cpp
)

add_executable(target_generator_service
  src/target_generator_service.cpp
  src/target_generator.cpp
)

target_link_libraries(turtlepi_nav_node
   ${catkin_LIBRARIES}
   ${Boost_LIBRARIES}
)

target_link_libraries(target_generator_service
   ${catkin_LIBRARIES}
)
