cmake_minimum_required(VERSION 3.8)
project(altair_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)


find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)
find_package(std_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(robotis_math REQUIRED)
find_package(Boost REQUIRED COMPONENTS system thread)

add_executable(arduino_data_node
  src/arduino_data/arduino_data_node.cpp
  src/arduino_data/arduino_data.cpp)

ament_target_dependencies(arduino_data_node PUBLIC rclcpp yaml_cpp_vendor sensor_msgs geometry_msgs std_msgs robotis_math tf2 tf2_ros Eigen3 Boost)
# target_link_libraries(arduino_data_node rclcpp yaml_cpp_vendor sensor_msgs geometry_msgs std_msgs robotis_math tf2 tf2_ros Eigen3)
# target_link_libraries(arduino_data_node Boost::system Boost::thread)

add_library(config
  src/altair_data/config.cpp
)
ament_target_dependencies(config
  yaml_cpp_vendor
  rclcpp
)

target_include_directories(config
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>")

# INSTALL LIBRARY
install(
  DIRECTORY include/
  DESTINATION include
)
install(
  TARGETS config
  EXPORT config
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

# INSTALL EXECUTABLES
install(TARGETS arduino_data_node
    DESTINATION lib/${PROJECT_NAME})

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
