^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_icp_slam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2018-09-21)
------------------
* Make catkin_lint clean
* Contributors: Jose Luis Blanco Claraco

0.1.7 (2018-09-20)
------------------

0.1.6 (2018-06-14)
------------------
* Fixed compilation with MRPT 1.5
* Fixed indents
* Fixed build with mrpt 2.0
* partial fix build w mrpt 2.0
* fix build in mrpt 2.0
* fix build; add optimized builds (-O3)
* fix build against mrpt 1.5 series
* use consistent cmake conventions for c++14
* all but mrpt_graphslam_2d compiling against mrpt2.0
* porting to mrpt2
* fix `#28 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/28>`_, compiling all nodes using -std=c++14
* CMake finds MRPT >=1.5 in branch master
* mrpt_icp_slam_2d: comment to adding-variables
* mrpt_icp_slam_2d: show in rviz and adding parameter to adjust rate in launch-files
  trajectory_update_rate: default 10 Hz
  trajectory_publish_rate: default 5 Hz
  [Reference] http://wiki.ros.org/hector_trajectory_server?distro=kinetic
* mrpt_icp_slam_2d: adding trajectory.
  [Reference] http://wiki.ros.org/hector_trajectory_server?distro=kinetic
* delete unused variable: curPDF.
* Merge pull request `#20 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/20>`_ from Logrus/master
  Fixes and cleanups for config files
* [mrpt_icp_slam_2d] Update description and dependencies of package.xml
  Add test for *.launch files.
* Move transforms to callbacks.
* Define C++11 avoiding direct manipulation of CXX_FLAGS
* catkin_lint error fixes
* Fix build against latest dev mrpt 1.5.0
* Contributors: Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Magnus Gärtner, Max Kuzmin, Vladislav Tananaev, tyuownu

0.1.5 (2016-11-18)
------------------
* Fix cmakelists typo
* Contributors: Nikos Koukis

0.1.3 (2016-09-27)
------------------

0.1.2 (2016-09-24)
------------------
* Make formatting conform to ROS C++ Style Guide.
* Output logs only in ROS.
* Add a guard for new mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl and mrpt_bridge::mrptToROSLoggerCallback functions for MRPT version less than 1.5.0.
* Add streaming of MRPT logs to ROS logs.
* Contributors: Logrus, Vladislav Tananaev

0.1.1 (2016-08-22)
------------------
* First public version, as a result of Vladislav Tananaev's GSoC2016 work.
* Contributors: Jose Luis Blanco, Logrus, Raphael Zack
