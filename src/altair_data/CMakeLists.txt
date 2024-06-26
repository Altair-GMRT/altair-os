cmake_minimum_required(VERSION 3.8)
project(altair_data)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

include_directories(${CMAKE_SOURCE_DIR}/include)


find_package(ament_cmake REQUIRED)
find_package(yaml_cpp_vendor REQUIRED)


add_library(config
  src/altair_data/config.cpp
)
ament_target_dependencies(config
  yaml_cpp_vendor
)
ament_export_targets(config
  HAS_LIBRARY_TARGET
)
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


if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
