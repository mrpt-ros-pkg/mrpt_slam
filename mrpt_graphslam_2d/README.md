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
- TODO - Upload and include Youtube demo video w/ ground-truth

## Algorithm demonstration

### Real-Time demo experiment - short loop

- A sample rosbag is included in the rosbags/demo_short_loop directory. To run
    this just launch the sr_graphslam_demo.launch file:

    `roslaunch mrpt_graphslam_2d sr_graphslam_demo.launch start_rviz:=True`

- Demo rosbag contains laserScan measurements (and odometry which is not usuable in the algorithm due to the topic type)


### Real-Time experiment - ground-truth data included

- A real-time experiment using a KUKA youbot and having a rough ground-truth
    estimate path is available
    [here](https://www.dropbox.com/sh/i672mt0uubw6muz/AADQiyNZuQc4pgBRT9choxsBa?dl=0).
    Ground-Truth estimation is provided using 2 ps3 cameras placed on the room
    ceiling. The cameras using the ar_sys ROS package are tracking Aruco
    markers that act as the workspace origin (static marker) and as the robot
    executing  SLAM (moving marker) respectively.
    To run this demo, just download the whole folder place it directly under
    the rosbags directory of the mrpt_graphslam_2d package and
    run the sr_graphslam_demo_gt.launch file.

    `roslaunch mrpt_graphslam_2d sr_graphslam_demo_gt.launch`

- One can also tinker with the aforementioned launchfile to enable/disable the
    different visualization features.

- Robot movement starts after ~60''. Due to different timestamps in the
    laserscans, odometry topics the algorithm feedback is lagging a bit compared
    to the ground-truth visualization

- sr_graphslam_demo_gt file uses the *pass_all_args* XML directive which is only
    available starting from the *ROS Jade* distribution. If you have an older one,
    just replace this with all the args in that file exclusively. For an
    example on how to do this, see the
    [sr_graphslam_demo.launch](https://github.com/mrpt-ros-pkg/mrpt_slam/blob/master/mrpt_graphslam_2d/launch/sr_graphslam_demo.launch)
    file

- Rosbag was recorded in the Mechanical Engineering School of the [National
	Technical University of Athens](http://www.mech.ntua.gr/en)

# Notes:

- Multi-Robot graphSLAM is not yet supported.
