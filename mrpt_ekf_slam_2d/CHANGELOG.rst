^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_ekf_slam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.8 (2018-09-21)
------------------
* Make catkin_lint clean
* Contributors: Jose Luis Blanco Claraco

0.1.7 (2018-09-20)
------------------

0.1.6 (2018-06-14)
------------------
* Fixed build with mrpt 2.0
* fix build; add optimized builds (-O3)
* fix build against mrpt 1.5 series
* use consistent cmake conventions for c++14
* all but mrpt_graphslam_2d compiling against mrpt2.0
* porting to mrpt2
  Fixes and cleanups for config files
* [mrpt_ekf_slam_2d] Update description in package.xml.
* [mrpt_ekf_slam_2d] Fix comment in launch file.
* [mrpt_ekf_slam_2d] Add missing dependencies to package.xml
  Add test of *.launch files for the node.
* Move transforms to callbacks.
* Move publishTF to callback.
* catkin_lint error fixes
* Contributors: Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Magnus Gärtner, Max Kuzmin, Vladislav Tananaev

0.1.5 (2016-11-18)
------------------
* Fix cmakelists typo
* Contributors: Nikos Koukis

0.1.3 (2016-09-27)
------------------
* Avoid gcc warnings with MRPT < 2.0.0
* fix build against mrpt < 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

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
* Contributors: Jose Luis Blanco, Logrus
