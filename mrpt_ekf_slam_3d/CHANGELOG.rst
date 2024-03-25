^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_ekf_slam_3d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2024-03-25)
-------------------

0.1.15 (2023-08-02)
-------------------

0.1.14 (2023-04-12)
-------------------
* Fix build against last mrpt 2.7.x
* Contributors: Jose Luis Blanco-Claraco

0.1.11 (2022-06-24)
-------------------
* Ported to tf2 and mrpt::ros1bridge
* Fixed all 2D and 3D EKF wrappers to work with mrpt 2.x API
* Update build dep to mrpt2
* Contributors: Jose Luis Blanco Claraco

0.1.10 (2019-10-05)
-------------------
* fix build against mrpt2
* Contributors: Jose Luis Blanco-Claraco

0.1.9 (2019-04-14)
------------------
* Fix build against MRPT 1.9.9
* Switch header guards to pragma once
* Contributors: Jose Luis Blanco-Claraco, Julian Lopez Velasquez, Vladislav Tananaev

0.1.8 (2018-09-21)
------------------
* Make catkin_lint clean
* Contributors: Jose Luis Blanco Claraco

0.1.7 (2018-09-20)
------------------

0.1.6 (2018-06-14)
------------------
* Fixed compilation with MRPT 1.5
* fix build in mrpt 2.0
* fix build; add optimized builds (-O3)
* use consistent cmake conventions for c++14
* Fixes and cleanups for config files
* [mrpt_ekf_slam_3d] Update description and dependencies of package.xml
  Add test for *.launch files.
* Move transforms to callbacks.
* Define C++11 avoiding direct manipulation of CXX_FLAGS
* catkin_lint error fixes
* Contributors: Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Logrus, Magnus Gärtner, Max Kuzmin, Vladislav Tananaev

0.1.5 (2016-11-18)
------------------
* Don't instruct cmake to search eigen3, mrpt has it
* Fix cmakelists typo
* avoid cmake errors if eigen is not found
* Contributors: Jose Luis Blanco-Claraco, Nikos Koukis

0.1.3 (2016-09-27)
------------------
* Avoid gcc warnings with MRPT < 2.0.0
* fix build against mrpt < 1.5.0
* Contributors: Jose-Luis Blanco-Claraco

0.1.2 (2016-09-24)
------------------
* Make formatting conform to ROS C++ Style Guide.
* fix eigen3 dependency problem
* Output logs only in ROS.
* Add a guard for new mrpt::ros1bridge::rosLoggerLvlToMRPTLoggerLvl and mrpt::ros1bridge::mrptToROSLoggerCallback functions for MRPT version less than 1.5.0.
* Add streaming of MRPT logs to ROS logs.
* Contributors: Jose Luis Blanco, Logrus, Vladislav Tananaev

0.1.1 (2016-08-22)
------------------
* First public version, as a result of Vladislav Tananaev's GSoC2016 work.
* Skip mrpt_ekf_slam_3d if version of MRPT is lower than 1.5.0.
* Contributors: Jose Luis Blanco, Logrus
