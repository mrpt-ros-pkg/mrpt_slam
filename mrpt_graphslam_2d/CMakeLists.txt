# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 3.1)
set (CMAKE_CXX_STANDARD 14)
project(mrpt_graphslam_2d)

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
  tf2
  multimaster_msgs_fkie
  )
## System dependencies are found with CMake's conventions
find_package(MRPT 1.5 REQUIRED gui obs maps opengl graphslam)
message(STATUS "MRPT_VERSION: ${MRPT_VERSION}")

if (MRPT_VERSION VERSION_LESS 2.0.0)
	add_compile_options(-Wno-deprecated-declarations)
endif()

if (CMAKE_COMPILER_IS_GNUCXX)
	# High level of warnings.
	# The -Wno-long-long is required in 64bit systems when including sytem headers.
	# The -Wno-variadic-macros was needed for Eigen3, StdVector.h
	add_compile_options(-Wall -Wno-long-long -Wno-variadic-macros)
	# Workaround: Eigen <3.4 produces *tons* of warnings in GCC >=6. See http://eigen.tuxfamily.org/bz/show_bug.cgi?id=1221
	if (NOT ${CMAKE_CXX_COMPILER_VERSION} LESS "6.0")
		add_compile_options(-Wno-ignored-attributes -Wno-int-in-bool-context)
	endif()
endif()

IF(CMAKE_COMPILER_IS_GNUCXX AND NOT CMAKE_BUILD_TYPE MATCHES "Debug")
  add_compile_options(-O3)
ENDIF()

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
#  INCLUDE_DIRS include
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
  multimaster_msgs_fkie
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

  set(SR_GRAPHSLAM_SRC
    src/CConnectionManager.cpp
  	src/common.cpp
    )

  set(MR_GRAPHSLAM_MERGER_SRC
  	src/CMapMerger.cpp
  	src/TNeighborAgentMapProps.cpp
  	)

  add_library(${PROJECT_NAME}
    ${SR_GRAPHSLAM_SRC}
    )
  add_executable(${PROJECT_NAME}_node
    ${SR_GRAPHSLAM_SRC}
    src/${PROJECT_NAME}_node.cpp
    )
  add_executable(${PROJECT_NAME}_mr_node
    ${SR_GRAPHSLAM_SRC}
    src/${PROJECT_NAME}_mr_node.cpp
    )
  add_executable(map_merger_node
  	${SR_GRAPHSLAM_SRC}
  	${MR_GRAPHSLAM_MERGER_SRC}
  	src/map_merger_node.cpp
  	)

  ## Specify libraries to link a library or executable target against
  target_link_libraries(${PROJECT_NAME}
    ${MRPT_LIBS}
    ${catkin_LIBRARIES}
    )
  target_link_libraries(${PROJECT_NAME}_node
    ${MRPT_LIBS}
    ${catkin_LIBRARIES}
    )
  target_link_libraries(${PROJECT_NAME}_mr_node
    ${MRPT_LIBS}
    ${catkin_LIBRARIES}
    )
  target_link_libraries(map_merger_node
  	${MRPT_LIBS}
  	${catkin_LIBRARIES}
  	)

  #############
  ## Install ##
  #############

  # Mark executables and/or libraries for installation
  install(
  	TARGETS ${PROJECT_NAME} ${PROJECT_NAME}_node ${PROJECT_NAME}_mr_node map_merger_node
    ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
    RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
    )

  # Mark other files for installation (e.g. launch and bag files, etc.)
  install(DIRECTORY
    launch
    config
    rviz
    rosbags
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
    )

  #############
  ## Testing ##
  #############

endif()
