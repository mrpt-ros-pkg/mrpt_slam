cmake_minimum_required(VERSION 3.8)
project(mrpt_graphslam_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 / ament packages are loaded first.
# MRPT packages are loaded AFTER the main target_link_libraries() calls.
# Their cmake configs can interfere with ament macros via transitive MRPT cmake
# files (mrpt_libros_bridge → mrpt-maps → system mrpt cmake).
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

if(CMAKE_COMPILER_IS_GNUCXX)
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  if(NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

include_directories(include)

# Single-robot node
add_executable(mrpt_graphslam_2d_node
    src/mrpt_graphslam_2d_node.cpp
    src/common.cpp
)

# ament_target_dependencies() was removed in ament_cmake 2.8.5 (Rolling).
# Use target_link_libraries() with modern cmake targets instead.
target_link_libraries(mrpt_graphslam_2d_node PRIVATE
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${tf2_geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${mrpt_msgs_TARGETS}
)

# Multi-robot node (TICKET-004)
add_executable(mrpt_graphslam_2d_mr_node
    src/mrpt_graphslam_2d_mr_node.cpp
    src/common.cpp
    src/CConnectionManager.cpp
)

target_link_libraries(mrpt_graphslam_2d_mr_node PRIVATE
    rclcpp::rclcpp
    ament_index_cpp::ament_index_cpp
    tf2::tf2
    tf2_ros::tf2_ros
    ${tf2_geometry_msgs_TARGETS}
    ${std_msgs_TARGETS}
    ${nav_msgs_TARGETS}
    ${sensor_msgs_TARGETS}
    ${geometry_msgs_TARGETS}
    ${visualization_msgs_TARGETS}
    ${mrpt_msgs_TARGETS}
)

# Map merger node — standalone map-merging utility (TICKET-004)
add_executable(map_merger_node
    src/map_merger_node.cpp
    src/common.cpp
    src/CConnectionManager.cpp
    src/CMapMerger.cpp
    src/TNeighborAgentMapProps.cpp
)

target_link_libraries(map_merger_node PRIVATE
    rclcpp::rclcpp
    ${nav_msgs_TARGETS}
    ${mrpt_msgs_TARGETS}
)

# Load MRPT packages after all target_link_libraries() calls.
# mrpt-graphslam is only available from the system PPA; prepend system cmake
# prefix so the correct mrpt-io (with CCompressedInputStream.h) is found first.
list(PREPEND CMAKE_PREFIX_PATH "/usr/lib/x86_64-linux-gnu/cmake")
find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_msgs_bridge REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-graphslam REQUIRED)

message(STATUS "MRPT_VERSION: ${mrpt-graphslam_VERSION}")

target_link_libraries(mrpt_graphslam_2d_node PRIVATE
    mrpt_msgs_bridge::mrpt_msgs_bridge
    mrpt::graphslam
    mrpt::gui
    mrpt_libros_bridge::mrpt_libros_bridge
)

target_link_libraries(mrpt_graphslam_2d_mr_node PRIVATE
    mrpt_msgs_bridge::mrpt_msgs_bridge
    mrpt::graphslam
    mrpt::gui
    mrpt_libros_bridge::mrpt_libros_bridge
)

target_link_libraries(map_merger_node PRIVATE
    mrpt_msgs_bridge::mrpt_msgs_bridge
    mrpt::graphslam
    mrpt::gui
    mrpt_libros_bridge::mrpt_libros_bridge
)

# TODO TICKET-004: fkie_multimaster_msgs removed — no ROS 2 equivalent
# find_package(fkie_multimaster_msgs REQUIRED)

install(TARGETS mrpt_graphslam_2d_node mrpt_graphslam_2d_mr_node map_merger_node
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
    launch config rviz rosbags
    DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node_startup   test/test_node_startup.py TIMEOUT 30)
  ament_add_pytest_test(test_launch_files   test/test_launch_files.py TIMEOUT 60)
  ament_add_pytest_test(test_parameters     test/test_parameters.py   TIMEOUT 30)
endif()

ament_package()
