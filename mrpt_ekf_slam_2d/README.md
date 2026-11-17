## Description

The sections below describe the API of this package for 2D (x,y,phi)
EKF-based SLAM. This method takes input measurements to landmarks in the
form of range-bearing observations.

## Further documentation

The ROS node mrpt_ekf_slam_2d is a wrapper for the C++
class[mrpt::slam::CRangeBearingKFSLAM2D](http://mrpt.ual.es/reference/devel/classmrpt_1_1slam_1_1_c_range_bearing_k_f_s_l_a_m2_d.html)
****, part of MRPT. Thus, check out the documentation of that class for
further details.

For the convention on coordinate frames see
[ROS REP 105: "Coordinate Frames for Mobile Platforms"](http://ros.org/reps/rep-0105.html).

## Usage

In order to use mrpt_ekf_slam_2d package it is necessary to install the
last [MRPT](http://www.mrpt.org/) build and
the[mrpt_navigation](http://wiki.ros.org/mrpt_navigation) (see also the
[tutorial](http://wiki.ros.org/mrpt_navigation/Tutorials/Installing)) .

## Example

The following video represents the EKF 2D SLAM in Rviz and MRPT GUI:

[Youtube](https://www.youtube.com/watch?v=tGMmi6Y6bL8)

The example of usage EKF 2D SLAM in online regime:



```
roslaunch mrpt_ekf_slam_2d ekf_slam_2d.launch
```

 The example of usage EKF 2D SLAM from .rawlog
file:



```
roslaunch mrpt_ekf_slam_2d ekf_slam_2d_rawlog.launch
```



## Nodes

### mrpt_ekf_slam_2d

The `mrpt_ekf_slam_2d` node takes in mrpt_msgs/ObservationRangeBearing
messages and builds a map with landmarks and robot pose. The map can be
retrieved via a ROS [topic](Topics).

#### Subscribed Topics

- `tf`
  ([tf/msg/tfMessage](https://docs.ros.org/en/rolling/p/tf/msg/tfMessage.html)):
  Transforms necessary to relate frames for laser, base, and odometry
  (see below)
- `landmark`
  ([mrpt_msgs/msg/ObservationRangeBearing](https://docs.ros.org/en/rolling/p/mrpt_msgs/msg/ObservationRangeBearing.html)):
  Landmarks to create the map

#### Published Topics

- `state_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the current state of the EKF
- `data_association_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the currently observed landmarks by robot

#### Parameters

- `~global_frame_id` (`string`, default: `"map"`): the frame
  attached to the map
- `~base_frame_id` (`string`, default: `"base_link"`): The frame
  attached to the mobile base
- `~odom_frame_id` (`string`, default: `"odom"`): The frame attached
  to the odometry system
- `~sensor_source` (`string`, default: `"scan"`): The name of topics
  with 2d laser scans or beacons (the name of topic should contain word
  `"scan"` or `"beacon"`)
- `~ini_filename` (`string`, default: `None`): The path to the ini file
  with MRPT settings for SLAM (see tutorial folder)
- `~rawlog_filename` (`string`, default: `None`): The path to rawlog
  file(if the rawlog file is not exists the SLAM starts in online
  regime)
- `~rawlog_play_delay` (`float`, default: `0.1`): Delay for replay SLAM
  from rawlog file.
- `~ellipse_scale` (`float`, default: `1`): The parameter for rescaling
  covariance ellipses visualization.

#### Required tf Transforms

- `<the frame attached to incoming scans>` → `base_link`: The relative
  pose of the laser sensor w.r.t. the robot. Usually a fixed value,
  broadcast periodically by a
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher),
  or a `tf` [static_transform_publisher](https://index.ros.org/p/tf).
- `base_link` → `odom`: The robot odometry, provided by the odometry
  system (e.g., the driver for the mobile base)

#### Provided tf Transforms

- `map` → `odom`: The current estimate of the robot's pose within the
  map frame
