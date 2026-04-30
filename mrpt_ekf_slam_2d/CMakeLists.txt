cmake_minimum_required(VERSION 3.8)
project(mrpt_ekf_slam_2d)

# Compiler settings for ROS2
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find ROS2 packages
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
find_package(mrpt_msgs_bridge REQUIRED)

# Find MRPT packages
find_package(mrpt-gui REQUIRED)
find_package(mrpt-slam REQUIRED)
find_package(mrpt-ros2bridge REQUIRED)

# Compiler options
if(CMAKE_COMPILER_IS_GNUCXX)
  # High level of warnings
  # The -Wno-long-long is required in 64bit systems when including system headers
  # The -Wno-variadic-macros was needed for Eigen3, StdVector.h
  add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
  # Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6
  if(NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
    add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
  endif()
endif()

if(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
endif()

###########
## Build ##
###########

include_directories(
  include
)

add_executable(mrpt_ekf_slam_2d
  src/mrpt_ekf_slam_2d.cpp
  src/mrpt_ekf_slam_2d_wrapper.cpp
  src/mrpt_ekf_slam_2d_app.cpp
)

# Dependencies for executable
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
  mrpt_msgs_bridge
)

# Link MRPT libraries to executable
target_link_libraries(mrpt_ekf_slam_2d
  mrpt::slam
  mrpt::gui
  mrpt::ros2bridge
)

#############
## Install ##
#############

# Install executable
install(TARGETS mrpt_ekf_slam_2d
  DESTINATION lib/${PROJECT_NAME}
)

# Install directories
install(DIRECTORY
  config
  launch
  tutorial
  rviz
  DESTINATION share/${PROJECT_NAME}
)

#############
## Testing ##
#############

if(BUILD_TESTING)
  find_package(ament_lint_auto REQUIRED)
  find_package(ament_cmake_pytest REQUIRED)
  find_package(ament_cmake_gtest REQUIRED)

  # Linting
  ament_lint_auto_find_test_dependencies()

  # Python smoke tests
  ament_add_pytest_test(test_launch_files test/test_launch_files.py
    TIMEOUT 60
  )

  ament_add_pytest_test(test_node_startup test/test_node_startup.py
    TIMEOUT 30
  )

  ament_add_pytest_test(test_parameters test/test_parameters.py
    TIMEOUT 30
  )

  # C++ unit tests (pure math — no ROS spin required)
  ament_add_gtest(test_ekf_slam_core test/test_ekf_slam_core.cpp)
  target_include_directories(test_ekf_slam_core PRIVATE include)
  target_link_libraries(test_ekf_slam_core mrpt::slam)
endif()

ament_package()
