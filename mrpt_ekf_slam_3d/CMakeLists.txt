# Catkin User Guide: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/user_guide.html
# Catkin CMake Standard: http://www.ros.org/doc/groovy/api/catkin/html/user_guide/standards.html
cmake_minimum_required(VERSION 2.8.3)
project(mrpt_ekf_slam_3d)
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

# Include Eigen3
# JLBC: From http://wiki.ros.org/jade/Migration#Eigen_CMake_Module_in_cmake_modules
find_package(Eigen3)
if(NOT EIGEN3_FOUND)
  # Fallback to cmake_modules
  find_package(cmake_modules REQUIRED)
  find_package(Eigen REQUIRED)
  set(EIGEN3_INCLUDE_DIRS ${EIGEN_INCLUDE_DIRS})
  set(EIGEN3_LIBRARIES ${EIGEN_LIBRARIES})  # Not strictly necessary as Eigen is head only
  # Possibly map additional variables to the EIGEN3_ prefix.
else()
  set(EIGEN3_INCLUDE_DIRS ${EIGEN3_INCLUDE_DIR})
endif()


#Include directories
INCLUDE_DIRECTORIES(
   ${CMAKE_SOURCE_DIR}/src
   ${CMAKE_CURRENT_BINARY_DIR}
)


catkin_package(
    CATKIN_DEPENDS
    DEPENDS Eigen 
    INCLUDE_DIRS ${EIGEN3_INCLUDE_DIRS}
    )

###########
## Build ##
###########

include_directories(
  ${catkin_INCLUDE_DIRS}
  ${Eigen_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIRS}
  include
)



add_executable(mrpt_ekf_slam_3d
    src/mrpt_ekf_slam_3d.cpp
    src/mrpt_ekf_slam_3d_wrapper.cpp
    src/mrpt_ekf_slam_3d_app.cpp
)

TARGET_LINK_LIBRARIES(mrpt_ekf_slam_3d ${MRPT_LIBS}
    ${catkin_LIBRARIES}  ${Eigen_LIBRARIES}
)

IF(${MRPT_VERSION} LESS 1.5.0)
  MESSAGE(WARNING "Target mrpt_ekf_slam_3d will not be built. It requires MRPT version 1.5.0 or higher, but you have ${MRPT_VERSION}.")
  set_target_properties(mrpt_ekf_slam_3d PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)
endif()
