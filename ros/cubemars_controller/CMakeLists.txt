cmake_minimum_required(VERSION 3.8)
project(cubemars_controller)

set(CMAKE_C_STANDARD 17)
set(CMAKE_CXX_STANDARD 17)

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(cubemars_msgs REQUIRED)

set(DRIVERS_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../../drivers)

add_executable(cubemars_controller_node
  src/cubemars_controller_node.cpp
  src/ak_mit_adapter.c
  ${DRIVERS_DIR}/src/ak_mit.c
)

add_executable(cubemars_feedback_listener_node
  src/cubemars_feedback_listener_node.cpp
  src/ak_mit_adapter.c
  ${DRIVERS_DIR}/src/ak_mit.c
)

target_include_directories(cubemars_controller_node PRIVATE
  src/
  ${DRIVERS_DIR}/inc
)

target_include_directories(cubemars_feedback_listener_node PRIVATE
  src/
  ${DRIVERS_DIR}/inc
)

ament_target_dependencies(cubemars_controller_node
  rclcpp
  cubemars_msgs
)

ament_target_dependencies(cubemars_feedback_listener_node
  rclcpp
  cubemars_msgs
)

install(TARGETS cubemars_controller_node cubemars_feedback_listener_node
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

ament_package()
