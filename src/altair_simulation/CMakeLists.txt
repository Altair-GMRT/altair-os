cmake_minimum_required(VERSION 3.5)
project(altair_simulation)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 14)
endif()
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -Wall -Wextra -Wpedantic")
endif()


find_package(ament_cmake REQUIRED)
find_package(gz_math_vendor REQUIRED)
find_package(gz-math REQUIRED)

find_package(gz_transport_vendor REQUIRED)
find_package(gz-transport REQUIRED)

find_package(gz_msgs_vendor REQUIRED)
find_package(gz-msgs REQUIRED)

find_package(gz_sim_vendor REQUIRED)
find_package(gz-sim REQUIRED)
find_package(xacro REQUIRED)
find_package(ros_gz_sim REQUIRED)
find_package(ros_gz_bridge REQUIRED)
# Needed in launch/gz_sim.launch.py.in
# set(GZ_SIM_VER ${gz-sim_VERSION_MAJOR})

gz_find_package(gflags
    REQUIRED
    PKGCONFIG gflags)
find_package(std_msgs REQUIRED)


# target_link_libraries(
#   # insert package name: 

#   gflags
#   gz-math::core
#   gz-msgs::core
#   gz-transport::core
# )
# ament_target_dependencies(
#   # insert package name:
  
#   std_msgs)

# target_include_directories(${PROJECT_NAME} PUBLIC
#   $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
#   $<INSTALL_INTERFACE:include>
# )

# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION include/${PROJECT_NAME}
# )

install(
  DIRECTORY launch urdf rviz worlds config meshes models
  DESTINATION share/${PROJECT_NAME}
)


ament_package()