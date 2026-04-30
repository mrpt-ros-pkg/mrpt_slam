cmake_minimum_required(VERSION 3.8)
project(mrpt_graphslam_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS 2 packages
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_ros REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(mrpt_msgs REQUIRED)
find_package(mrpt_msgs_bridge REQUIRED)

# Find MRPT packages
# mrpt-graphslam is only available from the system PPA.  Its headers
# transitively need mrpt-io, which must come from the same installation.
# Prepend the system cmake prefix so that the correct mrpt-io (with
# CCompressedInputStream.h) is found before the ROS-packaged version.
list(PREPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
find_package(mrpt-gui REQUIRED)
find_package(mrpt-graphslam REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

# TODO TICKET-004: fkie_multimaster_msgs removed — no ROS 2 equivalent
# find_package(fkie_multimaster_msgs REQUIRED)

message(STATUS "MRPT_VERSION: ${mrpt-graphslam_VERSION}")

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  if(NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

# Include directories
include_directories(include)

# Shared MR sources (used by both MR node and map_merger node)
set(MR_SOURCES
    src/CConnectionManager.cpp
    src/CMapMerger.cpp
    src/TNeighborAgentMapProps.cpp
)

# Single-robot node
add_executable(mrpt_graphslam_2d_node
    src/mrpt_graphslam_2d_node.cpp
    src/common.cpp
)

ament_target_dependencies(mrpt_graphslam_2d_node
    rclcpp
    ament_index_cpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    mrpt_msgs
    mrpt_msgs_bridge
)

target_link_libraries(mrpt_graphslam_2d_node
    mrpt::graphslam
    mrpt::gui
    mrpt::ros2bridge
)

# Multi-robot node (TICKET-004)
add_executable(mrpt_graphslam_2d_mr_node
    src/mrpt_graphslam_2d_mr_node.cpp
    src/common.cpp
    src/CConnectionManager.cpp
)

ament_target_dependencies(mrpt_graphslam_2d_mr_node
    rclcpp
    ament_index_cpp
    tf2
    tf2_ros
    tf2_geometry_msgs
    std_msgs
    nav_msgs
    sensor_msgs
    geometry_msgs
    visualization_msgs
    mrpt_msgs
    mrpt_msgs_bridge
)

target_link_libraries(mrpt_graphslam_2d_mr_node
    mrpt::graphslam
    mrpt::gui
    mrpt::ros2bridge
)

# Map merger node — standalone map-merging utility (TICKET-004)
add_executable(map_merger_node
    src/map_merger_node.cpp
    src/common.cpp
    src/CConnectionManager.cpp
    src/CMapMerger.cpp
    src/TNeighborAgentMapProps.cpp
)

ament_target_dependencies(map_merger_node
    rclcpp
    nav_msgs
    mrpt_msgs
)

target_link_libraries(map_merger_node
    mrpt::graphslam
    mrpt::gui
    mrpt::ros2bridge
)

# Install
install(TARGETS mrpt_graphslam_2d_node mrpt_graphslam_2d_mr_node map_merger_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch config rviz rosbags
    DESTINATION share/${PROJECT_NAME}
)

# Testing
if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node_startup   test/test_node_startup.py TIMEOUT 30)
  ament_add_pytest_test(test_launch_files   test/test_launch_files.py TIMEOUT 60)
  ament_add_pytest_test(test_parameters     test/test_parameters.py   TIMEOUT 30)
endif()

ament_package()
