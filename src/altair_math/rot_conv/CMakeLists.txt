cmake_minimum_required(VERSION 3.8)
project(rot_conv)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()


find_package(ament_cmake REQUIRED)

# Find and include Eigen
find_package(Eigen3 REQUIRED)

add_library(rot_conv
	src/rot_conv.cpp
)
ament_target_dependencies(rot_conv Eigen3)

target_include_directories(rot_conv
  PUBLIC
    "$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>"
    "$<INSTALL_INTERFACE:include/${PROJECT_NAME}>"
)

## Build the sample executable
add_executable(rot_conv_sample
	test/rot_conv_sample.cpp
)
target_link_libraries(rot_conv_sample rot_conv)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
## Rotation conversions test
ament_add_gtest(test_rot_conv test/test_rot_conv.cpp)

target_link_libraries(test_rot_conv
	rot_conv
	${GTEST_LIBRARIES}
        ${Eigen3_LIBRARIES}
)
endif()

install(
  DIRECTORY include/
  DESTINATION include/${PROJECT_NAME}
)

install(
  TARGETS rot_conv
  EXPORT export_${PROJECT_NAME}
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
)


ament_package()

