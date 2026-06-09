cmake_minimum_required(VERSION 3.8)
project(as2_3d_navigation_bringup)

if(NOT CMAKE_CXX_STANDARD)
  set(CMAKE_CXX_STANDARD 17)
endif()

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

find_package(ament_cmake REQUIRED)

# ---------------------------------------------------------------------------
# Standalone A* test (no ROS runtime dependency)
# Compiles astar3d.cpp inline so the binary runs without sourcing ROS.
# ---------------------------------------------------------------------------
find_package(eigen3_cmake_module REQUIRED)
find_package(Eigen3 REQUIRED)
find_package(as2_3d_map_interface REQUIRED)
find_package(as2_3d_map_server_plugin REQUIRED)

set(PLANNER_PKG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../as2_3d_path_planner_plugin)
set(MAP_IF_PKG_DIR  ${CMAKE_CURRENT_SOURCE_DIR}/../as2_3d_map_interface)
set(MAP_SRV_PKG_DIR ${CMAKE_CURRENT_SOURCE_DIR}/../as2_3d_map_server_plugin)
set(JPS3D_DIR       ${PLANNER_PKG_DIR}/thirdparty/jps3d)

# --- test_astar_artificial ---
add_executable(test_astar_artificial
  scripts/test_astar_artificial.cpp
  ${PLANNER_PKG_DIR}/src/astar3d.cpp
)

target_include_directories(test_astar_artificial PRIVATE
  ${PLANNER_PKG_DIR}/include
  ${MAP_IF_PKG_DIR}/include
  ${MAP_SRV_PKG_DIR}/include
)

target_link_libraries(test_astar_artificial Eigen3::Eigen)

# --- test_jps3d_artificial ---
add_executable(test_jps3d_artificial
  scripts/test_jps3d_artificial.cpp
  ${PLANNER_PKG_DIR}/src/astar3d.cpp
  ${PLANNER_PKG_DIR}/src/jps3d_planner.cpp
  ${JPS3D_DIR}/src/jps_planner/graph_search.cpp
  ${JPS3D_DIR}/src/jps_planner/jps_planner.cpp
)

target_include_directories(test_jps3d_artificial PRIVATE
  ${PLANNER_PKG_DIR}/include
  ${MAP_IF_PKG_DIR}/include
  ${MAP_SRV_PKG_DIR}/include
  ${JPS3D_DIR}/include
)

target_link_libraries(test_jps3d_artificial Eigen3::Eigen)

# --- test_minimum_snap ---
add_executable(test_minimum_snap
  scripts/test_minimum_snap.cpp
  ${PLANNER_PKG_DIR}/src/jps3d_planner.cpp
  ${PLANNER_PKG_DIR}/src/minimum_snap.cpp
  ${JPS3D_DIR}/src/jps_planner/graph_search.cpp
  ${JPS3D_DIR}/src/jps_planner/jps_planner.cpp
)

target_include_directories(test_minimum_snap PRIVATE
  ${PLANNER_PKG_DIR}/include
  ${MAP_IF_PKG_DIR}/include
  ${MAP_SRV_PKG_DIR}/include
  ${JPS3D_DIR}/include
)

target_link_libraries(test_minimum_snap Eigen3::Eigen)

install(
  TARGETS test_astar_artificial test_jps3d_artificial test_minimum_snap
  DESTINATION lib/${PROJECT_NAME}
)

# ---------------------------------------------------------------------------
# Launch / config / script files
# ---------------------------------------------------------------------------
install(
  DIRECTORY launch config scripts
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  set(ament_cmake_copyright_FOUND TRUE)
  set(ament_cmake_cpplint_FOUND TRUE)
  ament_lint_auto_find_test_dependencies()
endif()

ament_package()
