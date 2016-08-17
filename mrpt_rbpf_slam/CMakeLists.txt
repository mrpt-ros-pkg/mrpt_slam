cmake_minimum_required(VERSION 2.8.3)
project(mrpt_rbpf_slam)

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



add_executable(mrpt_rbpf_slam
    src/mrpt_rbpf_slam.cpp
    src/mrpt_rbpf_slam_wrapper.cpp
    src/mrpt_rbpf_slam_app.cpp
)

TARGET_LINK_LIBRARIES(mrpt_rbpf_slam ${MRPT_LIBS}
    ${catkin_LIBRARIES}
)

#############
## Install ##
#############


#############
## Testing ##
#############


