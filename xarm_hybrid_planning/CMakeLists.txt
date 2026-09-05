cmake_minimum_required(VERSION 3.16)
project(xarm_hybrid_planning)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

find_package(ament_cmake REQUIRED)
find_package(moveit_hybrid_planning REQUIRED)
find_package(moveit_core REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(pluginlib REQUIRED)
find_package(rclcpp REQUIRED)

add_library(replan_when_idle SHARED src/replan_when_idle.cpp)
target_include_directories(replan_when_idle PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(replan_when_idle
  moveit_hybrid_planning
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp)

add_library(nearest_waypoint_sampler SHARED src/nearest_waypoint_sampler.cpp)
target_include_directories(nearest_waypoint_sampler PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>)
ament_target_dependencies(nearest_waypoint_sampler
  moveit_hybrid_planning
  moveit_core
  moveit_msgs
  pluginlib
  rclcpp)

pluginlib_export_plugin_description_file(moveit_hybrid_planning replan_when_idle_plugin.xml)
pluginlib_export_plugin_description_file(moveit_hybrid_planning nearest_waypoint_sampler_plugin.xml)

install(
  TARGETS replan_when_idle nearest_waypoint_sampler
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin)
install(DIRECTORY include/ DESTINATION include)

ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
ament_export_dependencies(moveit_hybrid_planning moveit_core moveit_msgs pluginlib rclcpp)
ament_package()
