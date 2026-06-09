cmake_minimum_required(VERSION 3.8)
project(as2_3d_map_server_plugin)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_POSITION_INDEPENDENT_CODE ON)

# ament packages (their targets are handled by ament_target_dependencies)
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
  sensor_msgs
  tf2_ros
  tf2_eigen
  tf2_geometry_msgs
  as2_3d_map_interface
  octomap_ros
  octomap_msgs
  std_srvs
)

foreach(DEPENDENCY ${PROJECT_DEPENDENCIES})
  find_package(${DEPENDENCY} REQUIRED)
endforeach()

# Non-ament packages
find_package(octomap REQUIRED)
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)

include_directories(
  include
  include/${PROJECT_NAME}
  ${OCTOMAP_INCLUDE_DIRS}
)

add_library(${PROJECT_NAME} SHARED
  src/local_grid.cpp
  src/octomap_3d.cpp
  src/as2_3d_map_server_plugin.cpp
)

ament_target_dependencies(${PROJECT_NAME} ${PROJECT_DEPENDENCIES})

target_link_libraries(${PROJECT_NAME}
  as2_behaviors_path_planning::as2_behaviors_path_planning_plugin_base
  as2_3d_map_interface::as2_3d_map_interface
  ${OCTOMAP_LIBRARIES}
  Eigen3::Eigen
)

target_include_directories(${PROJECT_NAME} PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
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
