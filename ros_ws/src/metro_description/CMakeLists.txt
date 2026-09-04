cmake_minimum_required(VERSION 3.8)
project(metro_description)

find_package(ament_cmake REQUIRED)

install(
  DIRECTORY urdf meshes config
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_program(CHECK_URDF_EXECUTABLE check_urdf REQUIRED)
  add_test(
    NAME check_subway_v2_urdf
    COMMAND ${CHECK_URDF_EXECUTABLE}
      ${CMAKE_CURRENT_SOURCE_DIR}/urdf/subway_v2.urdf
  )
endif()

ament_package()
