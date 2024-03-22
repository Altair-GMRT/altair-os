cmake_minimum_required(VERSION 3.8)
project(altair_cpp_tests)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)


find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(altair_interfaces REQUIRED)


add_executable(rclnode_test
  src/rclnode_test.cpp
  src/altair_cpp_tests/rclnode_test_node.cpp
)
ament_target_dependencies(rclnode_test
  rclcpp 
  std_msgs
)
install(TARGETS
  rclnode_test
  DESTINATION lib/${PROJECT_NAME}
)


add_executable(custom_msg_test
  src/custom_msg_test.cpp
  src/altair_cpp_tests/custom_msg_test_node.cpp
)
ament_target_dependencies(custom_msg_test
  rclcpp 
  std_msgs
  altair_interfaces
)
install(TARGETS
  custom_msg_test
  DESTINATION lib/${PROJECT_NAME}
)


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
