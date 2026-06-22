cmake_minimum_required(VERSION 3.8)
project(cubemars_msgs)

find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MITCommand.msg"
  "msg/MITFeedback.msg"
  "srv/EnterMIT.srv"
  "srv/ExitMIT.srv"
  "srv/SetZero.srv"
)

ament_export_dependencies(rosidl_default_runtime)
ament_package()
