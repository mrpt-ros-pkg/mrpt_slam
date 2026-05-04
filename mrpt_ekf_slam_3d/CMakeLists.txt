cmake_minimum_required(VERSION 3.8)
project(mrpt_ekf_slam_3d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 / ament packages are loaded first.
# MRPT bridge packages are loaded AFTER the main target_link_libraries() call.
# See mrpt_ekf_slam_2d: system MRPT cmake configs can interfere with ament macros.
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

add_executable(mrpt_ekf_slam_3d
  src/mrpt_ekf_slam_3d.cpp
  src/mrpt_ekf_slam_3d_wrapper.cpp
  src/mrpt_ekf_slam_3d_app.cpp
)

# ament_target_dependencies() was removed in ament_cmake 2.8.5 (Rolling).
# Use target_link_libraries() with modern cmake targets instead.
target_link_libraries(mrpt_ekf_slam_3d PRIVATE
  rclcpp::rclcpp
  ${rclcpp_components_TARGETS}
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

find_package(mrpt_libros_bridge REQUIRED)
find_package(mrpt_msgs_bridge REQUIRED)
find_package(mrpt-gui REQUIRED)
find_package(mrpt-slam REQUIRED)

target_link_libraries(mrpt_ekf_slam_3d PRIVATE
  mrpt_msgs_bridge::mrpt_msgs_bridge
  mrpt::slam
  mrpt::gui
  mrpt_libros_bridge::mrpt_libros_bridge
)

install(TARGETS mrpt_ekf_slam_3d
  DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY
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

  ament_add_gtest(test_ekf_slam_3d_core test/test_ekf_slam_3d_core.cpp)
  target_include_directories(test_ekf_slam_3d_core PRIVATE include)
  target_link_libraries(test_ekf_slam_3d_core mrpt::slam)
endif()

ament_package()
