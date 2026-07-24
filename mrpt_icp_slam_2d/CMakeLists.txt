cmake_minimum_required(VERSION 3.8)
project(mrpt_icp_slam_2d)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# ROS 2 / ament packages are loaded first.
# MRPT packages are loaded AFTER the main target_link_libraries() call.
# Their cmake configs can interfere with ament macros via transitive MRPT cmake files.
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

add_executable(mrpt_icp_slam_2d
  src/mrpt_icp_slam_2d_wrapper.cpp
  src/mrpt_icp_slam_2d_app.cpp
)

# ament_target_dependencies() was removed in ament_cmake 2.8.5 (Rolling).
# Use target_link_libraries() with modern cmake targets instead.
target_link_libraries(mrpt_icp_slam_2d PRIVATE
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
)

find_package(mrpt-gui REQUIRED)
find_package(mrpt-slam REQUIRED)
find_package(mrpt_libros_bridge REQUIRED)

target_link_libraries(mrpt_icp_slam_2d PRIVATE
  mrpt::slam
  mrpt::gui
  mrpt_libros_bridge::mrpt_libros_bridge
)

install(TARGETS mrpt_icp_slam_2d
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
  find_package(launch_testing_ament_cmake REQUIRED)

  # uncrustify 0.78 (ament_cmake 2.8+/Rolling) crashes with exit code 70 on
  # the heavy-template headers in this package.
  list(APPEND AMENT_LINT_AUTO_EXCLUDE ament_cmake_uncrustify)
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

  find_package(mvsim QUIET)
  if(mvsim_FOUND)
    add_launch_test(test/test_mvsim_integration.launch.py
      TIMEOUT 60
    )
  else()
    message(STATUS "mvsim not found; skipping the ICP SLAM integration test")
  endif()
endif()

ament_package()
