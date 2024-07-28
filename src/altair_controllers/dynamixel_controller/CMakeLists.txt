cmake_minimum_required(VERSION 3.8)
project(dynamixel_controller)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)

# find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(std_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(robotis_math REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Boost REQUIRED)
find_package(yaml-cpp REQUIRED)
find_package(ament_index_cpp)
find_package(dynamixel_workbench)
find_package(dynamixel_workbench_toolbox)
# find_package(dynamixel_sdk REQUIRED)

add_executable(dynamixel_controller
  src/dynamixel_controller.cpp
  src/dynamixel_monitor.cpp
  src/dynamixel.cpp
  src/robot.cpp
  src/node.cpp
  )
target_include_directories(dynamixel_controller PUBLIC
$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
$<INSTALL_INTERFACE:include/${PROJECT_NAME}>)

ament_target_dependencies(dynamixel_controller rclcpp sensor_msgs geometry_msgs std_msgs robotis_math tf2 tf2_ros Boost yaml-cpp ament_index_cpp dynamixel_workbench dynamixel_workbench_toolbox)
target_link_libraries(dynamixel_controller yaml-cpp::yaml-cpp)

install(TARGETS dynamixel_controller
    DESTINATION lib/${PROJECT_NAME}
)

# install config files
install(FILES config/robot.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES config/dxl_init.yaml
  DESTINATION share/${PROJECT_NAME}/config)
install(FILES config/offset.yaml
  DESTINATION share/${PROJECT_NAME}/config)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  # the following line skips the linter which checks for copyrights
  # comment the line when a copyright and license is added to all source files
  set(ament_cmake_copyright_FOUND TRUE)
  # the following line skips cpplint (only works in a git repo)
  # comment the line when this package is in a git repo and when
  # a copyright and license is added to all source files
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
