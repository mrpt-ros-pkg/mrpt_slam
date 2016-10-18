# Description

Algorithm utilizes the MRPT mrpt-graphslam  library to execute single-robot
graphSLAM using information from odometry and LaserScan ROS topics.

mrpt_graphslam_2d is heavily based on the native MRPT *graphslam-engine*
application. Furthermore the command-line arguments offered by the latter can
be provided by the user as parameters in the ROS Parameter Server. See the
provided roslaunch files for examples of this. Also see the API of
mrpt_graphslam_2d available [here](// TODO - add it.)


## Additional information:
- mrpt-graphslam library: http://reference.mrpt.org/devel/namespacemrpt_1_1graphslam.html
- graphslam-engine application page: http://www.mrpt.org/list-of-mrpt-apps/application-graphslamengine/
- graphslam-engine documentation: // TODO add it here.


## Algorithm demonstration
- A prerecorded rosbag as well as instructions on how to run it, can be found
    in the demo/ directory
- Launch the launchfile that the demo/ directory contains. Tinker with the
	launch parameters if you want to modify the program behavior
- Rosbag was recorded in the Mechanical Engineering School of the [National
	Technical University of Athens](http://www.mech.ntua.gr/en)
