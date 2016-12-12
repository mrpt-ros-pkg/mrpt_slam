# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mrpt_graphslam_2d)

add_definitions(-std=c++11) # Use C++11

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    mrpt_bridge
    mrpt_msgs
    tf
    )
## System dependencies are found with CMake's conventions
find_package(MRPT REQUIRED
    base
    gui
    obs
    maps
    opengl
    graphslam)

if (MRPT_VERSION VERSION_LESS 2.0.0)
	set(CMAKE_CXX_FLAGS "-Wno-deprecated-declarations ${CMAKE_CXX_FLAGS}")
endif()


################################################
## Declare ROS messages, services and actions ##
################################################

################################################
## Declare ROS dynamic reconfigure parameters ##
################################################

###################################
## catkin specific configuration ##
###################################

catkin_package(
    INCLUDE_DIRS include
    #LIBRARIES mrpt_graphslam_2d
    CATKIN_DEPENDS
    roscpp
    rospy
    std_msgs
    sensor_msgs
    geometry_msgs
    nav_msgs
    mrpt_bridge
    mrpt_msgs
    tf
    #  DEPENDS system_lib
    )

###########
## Build ##
###########

## Specify additional locations of header files
## Your package locations should be listed before other locations
include_directories(
	include
  ${catkin_INCLUDE_DIRS}
	)

IF(${MRPT_VERSION} LESS 1.5.0)

    set(WARN_MSG "${WARN_MSG}\"${PROJECT_NAME}\" requires MRPT version 1.5.0 or higher ")
    set(WARN_MSG "${WARN_MSG} - Current MRPT version: ${MRPT_VERSION}\n")
    set(WARN_MSG "${WARN_MSG}Ignoring target \"${PROJECT_NAME}\"...")
    MESSAGE(WARNING ${WARN_MSG})
else()

## Declare a C++ executable
add_executable(${PROJECT_NAME}_node
    src/mrpt_graphslam_2d_node.cpp
    src/CGraphSlamResources.cpp
    src/TUserOptionsChecker_ROS.cpp
    )

## Specify libraries to link a library or executable target against
target_link_libraries(${PROJECT_NAME}_node
    ${MRPT_LIBS}
    ${catkin_LIBRARIES}
    )

#############
## Install ##
#############

# Mark executables and/or libraries for installation
install(TARGETS mrpt_graphslam_2d_node
  ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
  RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

# Mark other files for installation (e.g. launch and bag files, etc.)
install(DIRECTORY
  launch
  demo
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)

#############
## Testing ##
#############

endif()
