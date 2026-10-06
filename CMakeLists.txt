cmake_minimum_required(VERSION 3.8)
project(rsf_rviz_plugins)

find_package(ament_cmake REQUIRED)
find_package(pluginlib REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(rviz_rendering REQUIRED)
find_package(std_srvs REQUIRED)
find_package(rviz_default_plugins REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nmea_msgs REQUIRED)
find_package(diagnostic_msgs REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(nav2_msgs REQUIRED)
find_package(action_msgs REQUIRED)

set(CMAKE_AUTOMOC ON)
add_library(${PROJECT_NAME} SHARED
  include/rsf_rviz_plugins/nav_status_display.hpp
  include/rsf_rviz_plugins/odom_type_display.hpp
  src/dummy_obstacle_tool.cpp
  src/dummy_pedestrian_tool.cpp
  src/dummy_robot_team_tool.cpp
  src/nav_status_display.cpp
  src/odom_type_display.cpp
  src/rsf_status_panel.cpp
  src/view_switch_tool.cpp
  src/waypoint_navigation_panel.cpp
)
target_include_directories(${PROJECT_NAME} PRIVATE include)
ament_target_dependencies(${PROJECT_NAME}
  pluginlib rclcpp rviz_common rviz_rendering std_srvs rviz_default_plugins sensor_msgs visualization_msgs std_msgs
  nmea_msgs diagnostic_msgs ament_index_cpp nav2_msgs action_msgs
)
target_link_libraries(${PROJECT_NAME} Qt5::Widgets yaml-cpp)

pluginlib_export_plugin_description_file(rviz_common plugin_description.xml)
install(TARGETS ${PROJECT_NAME} LIBRARY DESTINATION lib)

ament_package()
