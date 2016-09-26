# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mrpt_ekf_slam_2d)
#c++ 11 standart
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
IF($ENV{VERBOSE})
	MESSAGE("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
endif()

#find catkin packages
find_package(catkin REQUIRED COMPONENTS roslib roscpp tf std_msgs nav_msgs sensor_msgs visualization_msgs mrpt_bridge dynamic_reconfigure)

#find mrpt packages
find_package(MRPT REQUIRED base gui obs slam)

if (MRPT_VERSION VERSION_LESS 2.0.0)
	set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
endif()


#Include directories
INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
   ${CMAKE_CURRENT_BINARY_DIR}
)


catkin_package( CATKIN_DEPENDS )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)



add_executable(mrpt_ekf_slam_2d
    src/mrpt_ekf_slam_2d.cpp
    src/mrpt_ekf_slam_2d_wrapper.cpp
    src/mrpt_ekf_slam_2d_app.cpp
)

TARGET_LINK_LIBRARIES(mrpt_ekf_slam_2d ${MRPT_LIBS}
    ${catkin_LIBRARIES}
)
