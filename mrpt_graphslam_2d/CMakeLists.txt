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
    src/mrpt_pause_shim.cpp
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
    src/mrpt_pause_shim.cpp
)

# Enable registration of MR-specific deciders (CFixedIntervalsNRD_MR,
# CLoopCloserERD_MR, etc.) only for the MR node.  The SR node must NOT pull in
# these deciders because (a) they require CConnectionManager symbols that are
# only linked into the MR and map-merger executables, and (b) using an MR
# decider with the SR engine would cause undefined behaviour at runtime.
target_compile_definitions(mrpt_graphslam_2d_mr_node PRIVATE
    MRPT_GRAPHSLAM_MR_DECIDERS
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
    src/mrpt_pause_shim.cpp
)

target_link_libraries(map_merger_node PRIVATE
    rclcpp::rclcpp
    ${nav_msgs_TARGETS}
    ${mrpt_msgs_TARGETS}
)

# Load MRPT packages after all target_link_libraries() calls. Keep every MRPT
# component in the same ROS installation: mixing the Ubuntu/PPA MRPT libraries
# in /usr with the ROS-bundled libraries gives the dynamic linker two
# ABI-compatible SONAMEs backed by different patch releases.
#
# Derive the prefix from the already-found rclcpp package rather than
# hard-coding a distro name.  rclcpp_DIR is set by find_package(rclcpp) to
# <prefix>/share/rclcpp/cmake; walking up three directories gives the root
# (e.g. /opt/ros/jazzy, /opt/ros/rolling, or a custom install space).
get_filename_component(_mrpt_ros_prefix "${rclcpp_DIR}" DIRECTORY)  # .../share/rclcpp
get_filename_component(_mrpt_ros_prefix "${_mrpt_ros_prefix}" DIRECTORY)  # .../share
get_filename_component(_mrpt_ros_prefix "${_mrpt_ros_prefix}" DIRECTORY)  # prefix root

set(
  MRPT_ROS_PREFIX "${_mrpt_ros_prefix}"
  CACHE PATH "Prefix containing the ROS MRPT installation (auto-derived from rclcpp)"
)
list(PREPEND CMAKE_PREFIX_PATH "${MRPT_ROS_PREFIX}")

if(NOT EXISTS
    "${MRPT_ROS_PREFIX}/lib/cmake/mrpt-graphslam/mrpt-graphslam-config.cmake"
)
  message(FATAL_ERROR
    "The ROS MRPT GraphSLAM component is missing from ${MRPT_ROS_PREFIX}. "
    "Install it with: sudo apt install ros-$ENV{ROS_DISTRO}-mrpt-libapps"
  )
endif()

find_package(
  mrpt_libros_bridge REQUIRED CONFIG
  PATHS "${MRPT_ROS_PREFIX}"
  NO_DEFAULT_PATH
)
find_package(mrpt_msgs_bridge REQUIRED)
find_package(
  mrpt-gui REQUIRED CONFIG
  PATHS "${MRPT_ROS_PREFIX}"
  NO_DEFAULT_PATH
)
find_package(
  mrpt-graphslam REQUIRED CONFIG
  PATHS "${MRPT_ROS_PREFIX}"
  NO_DEFAULT_PATH
)

if(NOT "${mrpt-gui_VERSION}" VERSION_EQUAL "${mrpt-graphslam_VERSION}")
  message(FATAL_ERROR
    "MRPT component version mismatch: mrpt-gui=${mrpt-gui_VERSION}, "
    "mrpt-graphslam=${mrpt-graphslam_VERSION}"
  )
endif()

# Guard against a transitive find_dependency() silently importing a second
# MRPT installation. Header-only targets have no location and are skipped.
get_property(_mrpt_imported_targets DIRECTORY PROPERTY IMPORTED_TARGETS)
foreach(_target IN LISTS _mrpt_imported_targets)
  if(NOT _target MATCHES "^mrpt::")
    continue()
  endif()

  get_target_property(_location "${_target}" IMPORTED_LOCATION)
  if(NOT _location)
    get_target_property(_location "${_target}" IMPORTED_LOCATION_NONE)
  endif()

  if(_location AND NOT _location MATCHES "^${MRPT_ROS_PREFIX}/")
    message(FATAL_ERROR
      "MRPT target ${_target} resolved outside the Jazzy installation: "
      "${_location}"
    )
  endif()
endforeach()

message(STATUS
  "MRPT GraphSLAM: ${mrpt-graphslam_VERSION} from ${mrpt-graphslam_DIR}"
)

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
  # uncrustify 0.78 (ament_cmake 2.8+/Rolling) crashes with exit code 70 on
  # the heavy-template headers in this package. Exclude it to avoid a false
  # test failure; cpplint and cppcheck still run for style/static analysis.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
  ament_lint_auto_find_test_dependencies()

  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_node_startup   test/test_node_startup.py TIMEOUT 30)
  ament_add_pytest_test(test_launch_files   test/test_launch_files.py TIMEOUT 60)
  ament_add_pytest_test(test_parameters     test/test_parameters.py   TIMEOUT 30)
endif()

ament_package()
