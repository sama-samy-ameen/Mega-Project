cmake_minimum_required(VERSION 3.8)
project(rov_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(std_msgs REQUIRED)  # <-- أضيفي هذا السطر هنا

rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/MotionCommands.msg"
  "msg/ThrustersSpeeds.msg"
  "msg/ActuatorsCommands.msg"
  "msg/RovCommand.msg"
  "msg/Guireader.msg"
  DEPENDENCIES std_msgs  # <-- وأضيفي هذا السطر هنا لربط الرسائل بـ std_msgs
)

ament_export_dependencies(rosidl_default_runtime)

ament_package()