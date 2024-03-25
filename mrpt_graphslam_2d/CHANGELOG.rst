^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
Changelog for package mrpt_graphslam_2d
^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^

0.1.16 (2024-03-25)
-------------------

0.1.15 (2023-08-02)
-------------------
* Fix build errors and warnings with latest mrpt2 versions
  (Closes: `#71 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/71>`_)
* Contributors: Jose Luis Blanco-Claraco

0.1.14 (2023-04-12)
-------------------
* 0.1.13
* fix version
* changelog
* Fix build against last mrpt 2.7.x
* chagelogs
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

0.1.11 (2022-06-24)
-------------------
* Ported to tf2 and mrpt::ros1bridge
* Fix build with mrpt2
* Update multimaster_msgs_fkie to fkie\_ prefix
* Merge branch 'master' of ssh://github.com/mrpt-ros-pkg/mrpt_slam
* Update URLs to https
* Update build dep to mrpt2
* Merge pull request `#65 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/65>`_ from Pillowline/patch-1
  setting trajectory publishers to latched
* setting trajectory publishers to latched
  So you can obtain data when running mrpt_graphslam with a bagfile, even after the playback has finished.
* Contributors: Jose Luis Blanco-Claraco, Pillowline

0.1.10 (2019-10-05)
-------------------
* fix build against current mrpt2
* fix build against mrpt2
* Contributors: Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco

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
* mrpt_graphslam_2d: Fix compilation warnings
* Remove installation of the demo directory from mrpt_graphslam_2d.
  It doesn't exist, so causes problems at the last stage of cmake.
  Signed-off-by: Chris Lalancette <clalancette@openrobotics.org>
* Contributors: Chris Lalancette, Jose Luis Blanco-Claraco, Nikos Koukis

0.1.6 (2018-06-14)
------------------
* Fixed compilation with MRPT 1.5
* Merge pull request `#41 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/41>`_ from MaxGsomGsom/master
  Fixed build with mrpt 2.0
* Fixed indents
* Fixed build with mrpt 2.0
* partial fix build w mrpt 2.0
* fix build in mrpt 2.0
* fix build; add optimized builds (-O3)
* fix build against mrpt 1.5 series
* Merge pull request `#32 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/32>`_ from bergercookie/master
  Correct prev_nodes_ICP error
* Correct prev_nodes_ICP member var
* mrpt_graphslam_2d: Add link to GitPitch slideshow
* mrpt_graphslam_2d: Validate user args in rename_rviz_topics.py
* use consistent cmake conventions for c++14
* all but mrpt_graphslam_2d compiling against mrpt2.0
* porting to mrpt2
* fix `#28 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/28>`_, compiling all nodes using -std=c++14
* CMake finds MRPT >=1.5 in branch master
* Merge pull request `#23 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/23>`_ from bergercookie/graphslam-devel
  mrpt_graphslam_2d:Add to ini config files, instructions
* mrpt_graphslam_2d:Fix broken links in README
* mrpt_graphslam_2d:Use template files and python script to generate rviz files for rosbag, gazebo runs
* mrpt_graphslam_2d:Add to the instructions for running graphSLAM from rosbags
* mrpt_graphslam_2d:Update graphslam_real_2.rviz file
* mrpt_graphslam_2d:Tweak config for real-time usage
* mrpt_graphslam_2d:Have .ini file specifically for gazebo simulations
* mrpt_graphslam_2d:CGraphSlamEngine_MR to have its own struct for config variables
* mrpt_graphslam_2d:Finish configuration for mr-slam with rosbags
* mrpt_graphslam_2d:Add initial setup and instructions for running mr-slam from rosbags
* mrpt_graphslam_2d:Correct sr_graphslam_demo_gt execution
* mrpt-graphslam:Modify parameters of mr-slam .ini file
* mrpt_graphslam:Add rviz file for 4 agents
* mrpt-graphslam:Remove deprecated .ini argument
* mrpt-graphslam_2d:Add using directives when needed
* Merge pull request `#22 <https://github.com/mrpt-ros-pkg/mrpt_slam/issues/22>`_ from bergercookie/graphslam-devel
  Add support for 2D multi-robot SLAM
* mrpt_graphslam_2d:Comply to MRPT changes
* Comply to new MRPT_LOG\_*_STREAM format
* Update rviz files, launchfiles
* mrpt_graphslam_2d: Correct error in visualization of robot orientation in rviz
* mrpt_graphslam_2d: Publish current robot_position to topic
* Update rviz file
* mrpt_graphslam_2d: Visualize compressed versions of images
* mrpt_graphslam_2d: Update documentation
* mrpt_graphslam_2d: Add local .gitignore
* mrpt_graphslam_2d: Skip first unecessary seconds of demo_short_loop rosbag
* Fix bug when running with more than 2 agents
  Bug occurred due to a dangling reference after rewriting the contents of a
  vector
* mrpt_graphslam_2d: Successful mr-graphslam real-time experiment
* Add configuration file for real-time mr-graphSLAM
* mrpt_graphslam_2d: Have topic_namespace of agent be the same as its name
* mrpt_graphslam_2d: Add rest of nodes in batches
  Knowing the transformation from own graph to neighbor's graph I
  integrate into own map the rest of the received nodes in batches of X
  nodes (default=4)
* Change names of files and classes *_CM -> *_MR
  Multi-robot SLAM is no longer (heavily) based on Condensed Measurements,
  thus the corresponding files/classes are named appropriately.
  MR: Multi-robot
* mrpt_graphslam_2d: Finish first working version of mr-graphSLAM
* mrpt-graphslam: Add more rviz files for gazebo
  Commit also temporarily deals with the segfault when running with more
  than 2 robots
* Finish first working version of map_merger node
  map_merger node subscribes to all the published maps and trajectories,
  fetches the results and upon user request computes an appropriate
  occupancy-grid map merging and joins all available maps and robot
  trajectories
* Start work on map_merger script
* mrpt_graphslam_2d: Class app to properly inherit from CGraphSlamHandler
  Add to mr-graphSLAM execution, various bug fixes
  Robot agents can now communicate LaserScans, modified node lists as well
  as condensed-measurements maps and upon successful matching integrate
  local map of other agent into own map.
* mrpt-graphslam-2d: Implement mr-graphSLAM communication system and structs
  Commit adds the necessary structures for basic communication of nodes,
  current LaserScan in a multi-robot graphSLAM application.
  Each graphSLAM agent publishes the last X nodes (node ID + pose) and its
  latest registered laser scan into corresponding topics under its own
  namespace. Furthermore each agent reads the aforementioned stats off the
  topics of all other agents that are currently running in the experiment.
  P.S. In the current implementation, each graphSLAM agent keeps a
  TNeighborAgentProps instance structure for each one of its found neighbors
  (*not* including self).
* mrpt_graphslam_2d: Add copyright string
* mrpt_graphslam_2d: Add to multi-robot CGraphSlamEngine
* mrpt_graphslam_2d: Implement conversion methods TSlamAgent <=> RosMaster
* mrpt_graphslam_2d: Have two different executables for sr, mr slam
  mrpt_graphslam_2d_node => single-robot graphSLAM
  mrpt_graphslam_2d_cm_node => multi-robot graphSLAM based on Condensed Measurements
* mrpt_graphslam_2d: Name classes, files in a consistent manner
* mrpt_graphslam_2d: Add ROS-specific, CondensedMeasurements-specific classes
  To facilitate polymorphism, task delegation, we implement class
  templates specific to the MR-graphSLAM strategy that inherit from
  mrpt-graphslam lib class templates
* mrpt_graphslam_2d: Add mr related class and executable
* mrpt_graphslam_2d: Add graphSLAM statistics topic -> feedback/graphslam_stats
* mrpt_graphslam_2d: Uncomment actual code snippet in main executable
* mrpt-graphslam-2d: Depend on multimaster_fkie pkg
* mrpt_graphslam_2d: Add draft version of CConnectionManager class
  CConnectionManager should be responsible for handling the inter-robot
  communication in an mr-slam setup. it basically provides a wrapper
  around the Multimaster package for finding other ROS masters in the same
  network over multicast
* mrpt_graphslam_2d: Abide to changes in mrpt-graphslam API
* mrpt-graphslam-2d: Add specialized versions of rviz files for Gazebo sim
* mrpt_graphslam_2d: Change script names
* mrpt_graphslam_2d: Add boilerplate code for multi-robot decider/optimizer classes.
  Multi-robot case is going to be implemented on the ROS side. Current
  commit adds code for the new multi-robot deciders/optimizer classes as
  well as verifying that the classes that are inputted by the user
  actually exist.
* Edit README.md file
* fix project name
* Define C++11 avoiding direct manipulation of CXX_FLAGS
* catkin_lint error fixes
* Contributors: Jose Luis Blanco, Jose Luis Blanco Claraco, Jose Luis Blanco-Claraco, Magnus Gärtner, Max Kuzmin, Nikos Koukis

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
