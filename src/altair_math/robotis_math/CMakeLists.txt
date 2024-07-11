cmake_minimum_required(VERSION 3.8)
project(robotis_math)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(Eigen3 REQUIRED)


add_library(${PROJECT_NAME}
   src/robotis_math_base.cpp
   src/robotis_trajectory_calculator.cpp
   src/minimum_jerk_trajectory.cpp
   src/minimum_jerk_trajectory_with_via_point.cpp
   src/fifth_order_polynomial_trajectory.cpp
   src/simple_trapezoidal_velocity_profile.cpp
   src/robotis_linear_algebra.cpp
   src/bezier_curve.cpp
   src/step_data_define.cpp
   src/preview_control.cpp
)

ament_target_dependencies(${PROJECT_NAME} rclcpp Eigen3)

target_include_directories(robotis_math
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)



install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS robotis_math
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)

# ament_export_targets(export_${PROJECT_NAME} HAS_LIBRARY_TARGET)
# ament_export_dependencies(some_dependency)

ament_export_include_directories("include/${PROJECT_NAME}")
ament_export_libraries(robotis_math)



ament_package()
