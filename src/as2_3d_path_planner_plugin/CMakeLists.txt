cmake_minimum_required(VERSION 3.5)
project(as2_3d_path_planner_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(PROJECT_DEPENDENCIES
  ament_cmake
  rclcpp
  pluginlib
  as2_core
  as2_msgs
  as2_behavior
  as2_behaviors_path_planning
  geometry_msgs
  nav_msgs
  visualization_msgs
  tf2_ros
  as2_3d_map_interface
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(octomap REQUIRED)

# ---------------------------------------------------------------------------
# JPS3D (vendored from https://github.com/KumarRobotics/jps3d, commit master)
# Only the jps_planner core is compiled — yaml-cpp (used by tests only) is
# NOT required.
# ---------------------------------------------------------------------------
set(JPS3D_DIR ${CMAKE_CURRENT_SOURCE_DIR}/thirdparty/jps3d)

include_directories(
  include
  include/${PROJECT_NAME}
  ${OCTOMAP_INCLUDE_DIRS}
  ${JPS3D_DIR}/include
)

add_library(${PROJECT_NAME} SHARED
  src/as2_3d_path_planner_plugin.cpp
  src/astar3d.cpp
  src/jps3d_planner.cpp
  src/minimum_snap.cpp
  ${JPS3D_DIR}/src/jps_planner/graph_search.cpp
  ${JPS3D_DIR}/src/jps_planner/jps_planner.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
  as2_behaviors_path_planning::as2_behaviors_path_planning_plugin_base
  as2_3d_map_interface::as2_3d_map_interface
  Eigen3::Eigen
  ${OCTOMAP_LIBRARIES}
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<BUILD_INTERFACE:${JPS3D_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

install(
  DIRECTORY include/
  DESTINATION include
)

install(
  TARGETS ${PROJECT_NAME}
  EXPORT export_${PROJECT_NAME}
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

ament_export_libraries(${PROJECT_NAME})
ament_export_include_directories(include)

pluginlib_export_plugin_description_file(as2_behaviors_path_planning plugins.xml)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
