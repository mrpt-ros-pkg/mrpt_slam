cmake_minimum_required(VERSION 3.8)
project(mrpt_ekf_slam_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Pure ROS 2 / ament packages — safe to load before ament_target_dependencies.
# MRPT bridge packages (mrpt_msgs_bridge, mrpt_libros_bridge, mrpt-gui, mrpt-slam)
# are loaded AFTER ament_target_dependencies because their cmake configs
# transitively pull in system MRPT cmake files that corrupt ament cmake macros.
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_components REQUIRED)
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

add_executable(mrpt_ekf_slam_2d
  src/mrpt_ekf_slam_2d.cpp
  src/mrpt_ekf_slam_2d_wrapper.cpp
  src/mrpt_ekf_slam_2d_app.cpp
)

# ament_target_dependencies must be called before any MRPT bridge find_package.
ament_target_dependencies(mrpt_ekf_slam_2d
  rclcpp
  rclcpp_components
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
)

# Load MRPT packages after ament_target_dependencies — their cmake configs
# corrupt ament cmake macros via transitive system MRPT cmake dependencies.
find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_msgs_bridge REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-slam REQUIRED)

target_link_libraries(mrpt_ekf_slam_2d
  mrpt_msgs_bridge::mrpt_msgs_bridge
  mrpt::slam
  mrpt::gui
  mrpt_libros_bridge::mrpt_libros_bridge
)

install(TARGETS mrpt_ekf_slam_2d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
  config
  launch
  tutorial
  rviz
  DESTINATION share/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  ament_lint_auto_find_test_dependencies()

  ament_add_pytest_test(test_launch_files test/test_launch_files.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_node_startup test/test_node_startup.py
    TIMEOUT 30
  )

  ament_add_pytest_test(test_parameters test/test_parameters.py
    TIMEOUT 30
  )

  ament_add_gtest(test_ekf_slam_core test/test_ekf_slam_core.cpp)
  target_include_directories(test_ekf_slam_core PRIVATE include)
  target_link_libraries(test_ekf_slam_core mrpt::slam)
endif()

option(ENABLE_INTEGRATION_TESTS "Build and register integration tests (slow, ~90 s)" OFF)

if(BUILD_TESTING AND ENABLE_INTEGRATION_TESTS)
  find_package(ament_cmake_pytest REQUIRED)
  ament_add_pytest_test(test_integration_rawlog
    test/test_integration_rawlog.py
    TIMEOUT 120
  )
endif()

ament_package()
