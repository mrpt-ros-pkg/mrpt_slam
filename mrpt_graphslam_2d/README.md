# General Information

[![GitPitch](https://gitpitch.com/assets/badge.svg)](https://gitpitch.com/bergercookie/mastert_present/master?grs=github)

Algorithm utilizes the MRPT mrpt-graphslam library to execute single,
multi-robot graphSLAM using information from odometry and LaserScan ROS topics.

mrpt\_graphslam\_2d is heavily based on the native MRPT *graphslam-engine*
application. Furthermore the command-line arguments offered by the latter can
be provided by the user as parameters in the ROS Parameter Server. See the
provided roslaunch files for examples of this. Also see the API of
mrpt\_graphslam\_2d available [here](http://docs.ros.org/lunar/api/mrpt_graphslam_2d/html/index.html)


## Additional information:
- [mrpt-graphslam library](http://reference.mrpt.org/devel/namespacemrpt_1_1graphslam.html)
- [graphslam-engine application page](http://www.mrpt.org/list-of-mrpt-apps/application-graphslamengine/)
- [graphslam-engine documentation](https://www.dropbox.com/s/u7phs612qf1l8bb/graphslam-engine-guide.pdf?dl=0)

## Single robot algorithm

### Real-Time experiment - short loop

- A sample rosbag is included in the `rosbags/demo_short_loop` directory. To run
    this just launch the sr_graphslam_demo.launch file:

    `roslaunch mrpt_graphslam_2d sr_graphslam_demo.launch start_rviz:=True`

- Demo rosbag contains Laser Scan measurements (and odometry which is not usuable in the algorithm due to the topic type)


### Real-Time experiment - ground-truth data included

- A real-time experiment using a KUKA youbot and having a rough ground-truth
    estimate path is available
    [here](https://www.dropbox.com/sh/i672mt0uubw6muz/AADQiyNZuQc4pgBRT9choxsBa?dl=0).
    Ground-Truth estimation is provided using 2 ps3 cameras placed on the room
    ceiling. The cameras using the ar_sys ROS package are tracking Aruco
    markers that act as the workspace origin (static marker) and as the robot
    executing  SLAM (moving marker) respectively.
    To run this demo, simply download the entire folder, place it directly under
    the `rosbags` directory of the mrpt_graphslam_2d package and run the
    sr_graphslam_demo_gt.launch file.

    `roslaunch mrpt_graphslam_2d sr_graphslam_demo_gt.launch`

- One can also tinker with the aforementioned launchfile to enable/disable the
    different visualization features, use another decider/optimizer class, or
    modify the .ini file to change the algorithm behavior.

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

## Multi-robot algorithm

Multi-robot support is provided via the
[csl_mr_slam](http://github.com/bergercookie/csl_mr_slam) package suite as well
as the use of multimaster_fkie packages utilized for communication across the
various ROS agents.

### Multi-robot real-time experiments - rosbag

A rosbag collected from real-robots can be found in the following link (more
rosbags to be added in the MRPT-2.0 branch):

- [multi_robot_graphSLAM_short](https://www.dropbox.com/sh/mxnij0jxvubyu2h/AADC8k6p-ZSq2nipGi4CiesFa?dl=0)

To run a demo using one of the corresponding rosbags download that directory,
place it directly under the `rosbags` directory of the mrpt_graphslam_2d
package and run the `run_mr_graphslam_demo.sh` script of the `csl_robots_gazebo
package`. The latter script (as is standard with the nodes of the
`csl_robots_gazebo` package reads its configuration parameters off the shell
environment at hand and adjusts its behavior accordingly. As an examaple users
can use the following command for running a multi-robot demo.

```sh
# see the script for configuration variables
rosrun csl_robots_gazebo run_mr_graphslam_demo.sh
```

**Warning**

- For this to work, you need to have the csl_mr_slam package suite (as well as the
packages that the latter depends on) in your catkin workspace.

- When executing multi-robot graphSLAM using either Gazebo (via the
    `csl_robots_gazebo` package or using measurements from the prerecorded
    rosbags, you have to, a priori, generate the necessary rviz files from the
    template files found in `$(rospack find mrpt_graphslam_2d)/rviz/templates`.
    To do that you have to run the `$(rospack find
    mrpt_graphslam_2d)/nodes/rename_rviz_topics.py` script which changes the
    necessary topic names based on the running computer's hostname. See
    documentation of the latter script for more on its usage.


### Multi-robot simulations in Gazebo - [csl_mr_slam](http://github.com/bergercookie/csl_mr_slam)

Multi-robot simulations are supported in the Gazebo Simulator via the
[csl_hw_setup](https://github.com/bergercookie/csl_mr_slam/tree/master/csl_hw_setup)
ROS package. An example of running such a simulation is given below.

![](https://media.giphy.com/media/l0Iydx6Dq3T7GSJVK/giphy.gif)

A complete example of executing multi-robot graphSLAM in the Gazebo simulation
environment is presented in the following video:

[![Multi-robot simulations](http://img.youtube.com/vi/4RKS2jrvsYE/0.jpg)](http://www.youtube.com/watch?v=4RKS2jrvsYE)

### Real-time multi-robot experiments

As with its single-robot variant, support for running the algorithm in a
real-time multi-robot setup is provided via the **csl_mr_slam/csl_hw_setup**
package.

An example of executing real-time multi-robot graphSLAM is given
[here](https://www.dropbox.com/s/zm2njljeprnsfaf/20170426_mr_graphslam_real_2.mp4?dl=0).

