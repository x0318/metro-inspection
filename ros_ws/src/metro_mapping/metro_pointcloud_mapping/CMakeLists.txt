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
find_package(nav_msgs REQUIRED)
find_package(PCL REQUIRED COMPONENTS common)
# PCL 1.12 exports these names in transitive link interfaces without making
# the imported targets visible in this directory on Ubuntu 22.04.
find_package(FLANN REQUIRED)
find_package(Qhull REQUIRED)
find_package(pcl_conversions REQUIRED)
find_package(rcl_interfaces REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rtabmap_msgs REQUIRED)
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

add_executable(optimized_cloud_saver src/optimized_cloud_saver_node.cpp)
target_include_directories(optimized_cloud_saver PRIVATE ${PCL_INCLUDE_DIRS})
target_link_libraries(optimized_cloud_saver voxel_map pcl_common)
ament_target_dependencies(optimized_cloud_saver
  pcl_conversions
  rcl_interfaces
  rclcpp
  sensor_msgs
  std_srvs
)

add_executable(cloud_gate src/cloud_gate_node.cpp)
ament_target_dependencies(cloud_gate
  rcl_interfaces
  rclcpp
  sensor_msgs
)

add_library(planar_motion_gate STATIC
  src/planar_motion_gate.cpp
)
target_include_directories(planar_motion_gate PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

add_executable(motion_cloud_gate src/motion_cloud_gate_node.cpp)
target_link_libraries(motion_cloud_gate planar_motion_gate)
ament_target_dependencies(motion_cloud_gate
  nav_msgs
  rcl_interfaces
  rclcpp
  sensor_msgs
)

add_library(map_graph_change_detector STATIC
  src/map_graph_change_detector.cpp
)
target_include_directories(map_graph_change_detector PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)
ament_target_dependencies(map_graph_change_detector
  geometry_msgs
  rtabmap_msgs
)

add_executable(map_data_gate src/map_data_gate_node.cpp)
target_link_libraries(map_data_gate map_graph_change_detector)
ament_target_dependencies(map_data_gate
  rcl_interfaces
  rclcpp
  rtabmap_msgs
)

install(TARGETS
  cloud_accumulator
  cloud_gate
  map_data_gate
  motion_cloud_gate
  optimized_cloud_saver
  DESTINATION lib/${PROJECT_NAME}
)
install(PROGRAMS scripts/check_rtabmap_graph.py
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
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  ament_add_gtest(test_voxel_map test/test_voxel_map.cpp)
  target_link_libraries(test_voxel_map voxel_map)

  ament_add_gtest(test_pcd_writer test/test_pcd_writer.cpp)
  target_link_libraries(test_pcd_writer voxel_map)

  ament_add_gtest(
    test_map_graph_change_detector test/test_map_graph_change_detector.cpp
  )
  target_link_libraries(
    test_map_graph_change_detector map_graph_change_detector
  )

  ament_add_gtest(test_planar_motion_gate test/test_planar_motion_gate.cpp)
  target_link_libraries(test_planar_motion_gate planar_motion_gate)

  ament_add_pytest_test(
    test_graph_slam_config test/test_graph_slam_config.py
  )
  ament_add_pytest_test(
    test_rtabmap_graph_checker test/test_rtabmap_graph_checker.py
  )
endif()

ament_package()
