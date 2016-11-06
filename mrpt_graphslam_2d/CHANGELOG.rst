^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_graphslam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.4 (2016-11-06)
------------------
* Add install targets to CMake.
* mrpt_graphslam_2d: Init MR-SLAM configuration
  Commit adds boilerplate code for:
  - Launchfile with nested topic and TF groups for manipulating more
  robotic agents in a consistent manner
  - New .rviz file for MR-SLAM
* mrpt_graphslam_2d: queue_size as a private member
* mrpt_graphslam_2d: Cleanup CMakeLists file, add catkin_INCLUDE_DIRS
* Add demo workspace picture
* mrpt_graphslam_2d: Make changes to README instructions and app launchfiles
* mrpt_graphslam_2d: Initialize demo folder, Modify README
* mrpt_graphslam_2d: Add to the feedback results
* mrpt_graphslam_2d: Initialize feedback topics
  Provide feedback information that can be accessed via ROS Topics. These
  utilize the CGraphslamEngine API and include the following:
  - Latest robot pose
  - Estimated path trajectory
* mprt_graphslam_2d: Use m\_ prefix for class private vars
* mrpt_graphslam_2d: Save result files after execution
* Add README file.
* mrpt_graphslam_2d:Add launchfile, configfile
* mrpt_graphslam_2d: Initialize ROS wrapper for mrpt-graphslam
  Commit includes boilerplate code for running graphSLAM using the
  mrpt-graphslam library.
  The following should be noted:
  - mrpt_graphslam_2d is heavily based on the native MRPT
  graphslam-engine_app application.
  - graphslam-engine_app command line arguments correspond to parameters
  in the /graphslam_engine namespace of the ROS parameter server and can
  be set either by an external launchfile or by dirctly by the user.
* Contributors: Logrus, Nikos Koukis

0.1.3 (2016-09-27)
------------------

0.1.2 (2016-09-24)
------------------

0.1.1 (2016-08-22)
------------------
