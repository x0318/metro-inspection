cmake_minimum_required(VERSION 3.8)
project(metro_pointcloud_mapping)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# PCL 1.12 exports these names in transitive link interfaces without making
# the imported targets visible in this directory on Ubuntu 22.04.
find_package(FLANN REQUIRED)
find_package(Qhull REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_eigen REQUIRED)
find_package(tf2_ros REQUIRED)

add_definitions(${PCL_DEFINITIONS})

add_library(voxel_map STATIC
  src/pcd_writer.cpp
  src/voxel_map.cpp
)
target_include_directories(voxel_map PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(cloud_accumulator src/cloud_accumulator_node.cpp)
target_include_directories(cloud_accumulator PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(cloud_accumulator voxel_map pcl_common)
ament_target_dependencies(cloud_accumulator
  geometry_msgs
  pcl_conversions
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_srvs
  tf2
  tf2_eigen
  tf2_ros
)

install(TARGETS cloud_accumulator
  DESTINATION lib/${PROJECT_NAME}
)
install(DIRECTORY include/
  DESTINATION include
)
install(DIRECTORY config launch
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map voxel_map)

  ament_add_gtest(test_pcd_writer test/test_pcd_writer.cpp)
  target_link_libraries(test_pcd_writer voxel_map)
endif()

ament_package()
