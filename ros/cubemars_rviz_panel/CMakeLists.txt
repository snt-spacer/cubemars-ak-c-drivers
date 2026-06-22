cmake_minimum_required(VERSION 3.8)
project(cubemars_rviz_panel)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_AUTOMOC ON)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rviz_common REQUIRED)
find_package(pluginlib REQUIRED)
find_package(cubemars_msgs REQUIRED)
find_package(Qt5 REQUIRED COMPONENTS Widgets Core)

add_library(cubemars_rviz_panel SHARED
  src/cubemars_panel.cpp
  src/motor_widget.cpp
)

target_include_directories(cubemars_rviz_panel PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/src>
  $<INSTALL_INTERFACE:include>
)

ament_target_dependencies(cubemars_rviz_panel
  rclcpp
  rviz_common
  pluginlib
  cubemars_msgs
)

target_link_libraries(cubemars_rviz_panel
  Qt5::Widgets
  Qt5::Core
)

pluginlib_export_plugin_description_file(rviz_common cubemars_panel_plugin.xml)

install(TARGETS cubemars_rviz_panel
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

install(FILES cubemars_panel_plugin.xml
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
