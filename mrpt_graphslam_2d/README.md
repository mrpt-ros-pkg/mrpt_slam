# Description

Algorithm utilizes the MRPT mrpt-graphslam  library to execute single-robot
graphSLAM using information from odometry and LaserScan ROS topics.

mrpt\_graphslam\_2d is heavily based on the native MRPT *graphslam-engine*
application. Furthermore the command-line arguments offered by the latter can
be provided by the user as parameters in the ROS Parameter Server. See the
provided roslaunch files for examples of this. Also see the API of
mrpt\_graphslam\_2d available [here](// TODO - add it.)


## Additional information:
- [mrpt-graphslam library](http://reference.mrpt.org/devel/namespacemrpt_1_1graphslam.html)
- [graphslam-engine application page](http://www.mrpt.org/list-of-mrpt-apps/application-graphslamengine/)
- [graphslam-engine documentation](https://www.dropbox.com/s/u7phs612qf1l8bb/graphslam-engine-guide.pdf?dl=0)


## Algorithm demonstration
- A prerecorded rosbag as well as instructions on how to run it, can be found
    in the demo/ directory
- Launch the launchfile that the demo/ directory contains.

    `export ROS_IS_SIMULATION=1 && roslaunch <path_to_launchfile>`

    Tinker with the launch parameters if you want to modify the program behavior
- Optionally visualize the graphSLAM execution in rviz:

  `rosrun rviz rviz -d  \$(rospack find mrpt_graphslam_2d)/misc/default.rviz`

- Rosbag was recorded in the Mechanical Engineering School of the [National
	Technical University of Athens](http://www.mech.ntua.gr/en)
