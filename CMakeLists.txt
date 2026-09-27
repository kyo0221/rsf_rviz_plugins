cmake_minimum_required(VERSION 3.8)
project(rsf_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(std_srvs REQUIRED)

set(CMAKE_AUTOMOC ON)
add_library(waypoint_navigation_panel SHARED
  include/rsf_rviz_plugins/waypoint_navigation_panel.hpp
  src/waypoint_navigation_panel.cpp
)
target_include_directories(waypoint_navigation_panel PRIVATE include)
ament_target_dependencies(waypoint_navigation_panel
  pluginlib rclcpp rviz_common std_srvs
)
target_link_libraries(waypoint_navigation_panel Qt5::Widgets)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS waypoint_navigation_panel LIBRARY DESTINATION lib)

ament_package()
