^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_ekf_slam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

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
