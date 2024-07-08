cmake_minimum_required(VERSION 3.8)
project(rc_utils)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(rclcpp REQUIRED)
find_package(tf2 REQUIRED)

## Build the rc_utils library
add_library(rc_utils
	src/angle_estimator.cpp
	src/attitude_estimator.cpp
	src/conicfit.cpp
	src/planefit.cpp
	src/cyclicwarp.cpp
	src/golay.cpp
	src/kalmanfilter.cpp
)

ament_target_dependencies(rc_utils PUBLIC Eigen3 tf2)


target_include_directories(rc_utils
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

################
## Unit tests ##
################
if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)

## Attitude estimator test
ament_add_gtest(test_attitude_estimator
	test/test_attitude_estimator.cpp
)
target_link_libraries(test_attitude_estimator
	rc_utils
)

## Conic fit test
ament_add_gtest(test_conicfit
	test/test_conicfit.cpp
)
target_link_libraries(test_conicfit
	rc_utils
)

## Plane fit test
ament_add_gtest(test_planefit
	test/test_planefit.cpp
)
target_link_libraries(test_planefit
	rc_utils
)

## FIR filter test
ament_add_gtest(test_firfilter
	test/test_firfilter.cpp
)
target_link_libraries(test_firfilter
	rc_utils
)

## Golay test
ament_add_gtest(test_golay
	test/test_golay.cpp
)
target_link_libraries(test_golay
	rc_utils
)

## Test math utilities
ament_add_gtest(test_math_util
	test/test_math_util.cpp
)
target_link_libraries(test_math_util
	rc_utils
)

## Test miscellaneous utilities
ament_add_gtest(test_misc_util
	test/test_misc_util.cpp
)
target_link_libraries(test_misc_util
	rc_utils
)
endif()
#############
## Install ##
#############

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS rc_utils
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()