^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_graphslam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.5 (2016-11-18)
------------------
* mrpt_graphslam_2d: Correct syntax in README file
* mrpt_graphslam_2d: Complete the demo rviz, launch files
  Finish setting up the demos-related files.
  Setup a hierarchy of launchfiles with each each one delegating the
  corresponding tasks to the next one with the sr_graphslam.launch as the
  final link in this chain. This should make up for an easier maintenance
  of the whole setup later on.
* Renamed demo bagfile
* Be consistent with rviz, launchfile names
* Readd demo_short_loop bag
* mrpt_graphslam_2d: Add demo_gt launchfile for launching demo rosbag
* Skip mrpt_graphslam_2d compilation if MRPT version < 1.5.0
* mrpt_graphslam_2d: Add rviz file for complete single-robot SLAM experiment
* mrpt_graphslam_2d: Use tf2 for all tf transformations.
  Commit also includes the following:
  - Introduction of the "anchor node", that is the frame that (a specific)
  robot trajectory starts from, which should also differ from the world
  frame in a multi-robot setup.
  - Odometry input messages are expected to be of type nav_msgs::Odometry,
  instead of the custom msg Pose2DStamped used so far
* mrpt_graphslam_2d: Make changes to graphslam.launch file
* mrpt_graphslam_2d: Add to the launchfiles
* Contributors: Nikos Koukis

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
