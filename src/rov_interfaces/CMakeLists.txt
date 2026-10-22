cmake_minimum_required(VERSION 3.8)
project(rov_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommands.msg"
  "msg/ThrustersSpeeds.msg"
  "msg/ActuatorsCommands.msg"
  "msg/Guireader.msg"
  "msg/ObjectData.msg"
  "msg/RovCommand.msg"
  "msg/AutonomousCommand.msg"
  "msg/ModelReadings.msg"
  DEPENDENCIES geometry_msgs std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()