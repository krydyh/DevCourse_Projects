cmake_minimum_required(VERSION 3.0.2)
project(final_project)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

include_directories(
# include
 ${catkin_INCLUDE_DIRS}
)

find_package(gazebo REQUIRED)
find_package(ignition-common3 REQUIRED)
find_package(ignition-math6 REQUIRED)
find_package(sdformat9 REQUIRED)
find_package(catkin REQUIRED COMPONENTS
  gazebo_plugins
  gazebo_ros
  roscpp
  std_msgs
  smach
  smach_ros
  actionlib_msgs
  geometry_msgs
  actionlib
  tf2_geometry_msgs
  tf2_ros
  smach_msgs
)

catkin_package()

include_directories(${GAZEBO_INCLUDE_DIRS})
include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)
link_directories(${GAZEBO_LIBRARY_DIRS})
list(APPEND CMAKE_CXX_FLAGS "${GAZEBO_CXX_FLAGS}")

add_executable(moving_node src/moving.cpp)
target_link_libraries(moving_node ${catkin_LIBRARIES})
target_link_libraries(moving_node ${GAZEBO_LIBRARIES} 
${IGNITION-COMMON_LIBRARIES}
${IGNITION-MATH_LIBRARIES}
${SDFORMAT_LIBRARIES})

add_executable(custom_follow_waypoints_node src/custom_follow_waypoints.cpp)
target_link_libraries(custom_follow_waypoints_node ${catkin_LIBRARIES})
target_link_libraries(custom_follow_waypoints_node ${GAZEBO_LIBRARIES} 
${IGNITION-COMMON_LIBRARIES} 
${IGNITION-MATH_LIBRARIES} 
${SDFORMAT_LIBRARIES})


