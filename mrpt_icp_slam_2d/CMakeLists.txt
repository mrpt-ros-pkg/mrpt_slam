# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mrpt_icp_slam_2d)
# Load catkin and all dependencies required for this package
# TODO: remove all from COMPONENTS that are not catkin packages.
#c++ 11 standart
set(CMAKE_CXX_FLAGS "-std=c++11 ${CMAKE_CXX_FLAGS}")
IF($ENV{VERBOSE})
	MESSAGE("CMAKE_CXX_FLAGS: ${CMAKE_CXX_FLAGS}")
endif()

#find catkin packages
find_package(catkin REQUIRED COMPONENTS roslib roscpp tf std_msgs nav_msgs sensor_msgs visualization_msgs mrpt_bridge dynamic_reconfigure)

#find mrpt packages
find_package(MRPT REQUIRED base gui obs slam)

#Include directories
INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
)


catkin_package( CATKIN_DEPENDS)

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  include
)



add_executable(mrpt_icp_slam_2d
    src/mrpt_icp_slam_2d_wrapper.cpp
    src/mrpt_icp_slam_2d_app.cpp
)

TARGET_LINK_LIBRARIES(mrpt_icp_slam_2d ${MRPT_LIBS}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############


