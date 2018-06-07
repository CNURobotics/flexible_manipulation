cmake_minimum_required(VERSION 2.8.3)
project(flexible_manipulation_moveit_capabilities)

#set(CMAKE_BUILD_TYPE Debug)
set (CMAKE_CXX_STANDARD 11)

find_package( Boost REQUIRED COMPONENTS system thread )

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
  cmake_modules
  geometry_msgs
  moveit_msgs
  moveit_ros_move_group
  pluginlib
  roscpp
  flexible_manipulation_msgs
  tf
)

catkin_package(
  INCLUDE_DIRS include
  LIBRARIES flexible_manipulation_moveit_capabilities
  CATKIN_DEPENDS flexible_manipulation_msgs geometry_msgs moveit_msgs moveit_ros_move_group pluginlib roscpp
#  DEPENDS system_lib
)

include_directories(
  include
  ${catkin_INCLUDE_DIRS}
)

add_library(flexible_manipulation_moveit_capabilities
  src/helper/continuous_plan_execution.cpp
  src/extended_capabilities/move_group_extended_action_capability.cpp
  src/extended_capabilities/get_planning_group_config_action_capability.cpp
  src/extended_capabilities/octomap_access_capability.cpp
  src/extended_capabilities/continuous_replanning_capability.cpp
  src/default_capabilities/apply_planning_scene_capability.cpp
  src/default_capabilities/clear_octomap_capability.cpp
  src/default_capabilities/execute_known_trajectory_capability.cpp
  src/default_capabilities/get_cartesian_path_capability.cpp
  src/default_capabilities/get_planning_scene_capability.cpp
  src/default_capabilities/kinematics_capability.cpp
  src/default_capabilities/move_group_plan_capability.cpp
  src/default_capabilities/query_planners_capability.cpp
  src/default_capabilities/state_validation_capability.cpp
)

target_link_libraries(flexible_manipulation_moveit_capabilities ${catkin_LIBRARIES} ${Boost_LIBRARIES})

add_dependencies(flexible_manipulation_moveit_capabilities ${catkin_EXPORTED_TARGETS})

install(TARGETS flexible_manipulation_moveit_capabilities
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY include/ DESTINATION include)

install(FILES
  flexible_manipulation_moveit_capabilities_plugin_description.xml
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION})
