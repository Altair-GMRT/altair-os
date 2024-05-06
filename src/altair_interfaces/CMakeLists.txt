cmake_minimum_required(VERSION 3.8)
project(altair_interfaces)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(altair_data)


rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/TestMessage.msg"
  "msg/JointTorque.msg"
  "msg/JointPosition.msg"
  "msg/JointVelocity.msg"
  "msg/JointSensor.msg"
  "msg/Inertial.msg"
)
ament_export_dependencies(rosidl_default_runtime)
rosidl_get_typesupport_target(cpp_typesupport_target
  ${PROJECT_NAME} 
  rosidl_typesupport_cpp
)


add_executable(app_launcher
  src/app_launcher.cpp
  src/altair_interfaces/app_launcher_node.cpp
)
ament_target_dependencies(app_launcher
  rclcpp
  altair_data
)
install(TARGETS
  app_launcher
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
