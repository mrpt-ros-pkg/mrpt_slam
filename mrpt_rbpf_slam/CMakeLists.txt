cmake_minimum_required(VERSION 2.8.3)
project(mrpt_rbpf_slam)

add_definitions(-std=c++11) # Use C++11

#find catkin packages
find_package(catkin REQUIRED COMPONENTS
  roslib
  roscpp
  tf
  std_msgs
  nav_msgs
  sensor_msgs
  visualization_msgs
  mrpt_bridge 
  dynamic_reconfigure
  roslaunch)

#find mrpt packages
find_package(MRPT REQUIRED base gui obs slam)

#Include directories
INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
   ${CMAKE_CURRENT_BINARY_DIR}
)


catkin_package(
	CATKIN_DEPENDS
	nav_msgs sensor_msgs std_msgs
)

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

# Mark executables and/or libraries for installation
install(TARGETS mrpt_rbpf_slam
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  tutorial
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############
# Add testing for *.launch files
roslaunch_add_file_check(launch)
