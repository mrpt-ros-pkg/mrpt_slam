## Description

The sections below describe the API of the package. The interface is
similar to gmapping (<http://wiki.ros.org/gmapping>) but the package
supports different particle filter algorithms, range-only SLAM and can
work with several grid maps simultaneously.

This package supports the following Particle Filter SLAM algorithms
(more details at
<http://www.mrpt.org/tutorials/slam-algorithms/rbpf-slam_algorithms/>):

- Sequential Importance Resampling -- SIR with the "standard" proposal
  distribution (enum value: pfStandardProposal)
- Auxiliary Particle Filter -- APF with the "standard" proposal
  distribution (enum value: pfAuxiliaryPFStandard)
- Optimal proposal distribution (enum value: pfOptimalProposal)
- Approximate Optimal Sampling -- A rejection sampling-based
  approximation to the optimal proposal when a closed-form expression is
  not available (enum value: pfAuxiliaryPFOptimal)

These algorithms allow building **grid map or range-only map**.

## Further documentation

The ROS node `mrpt_rbpf_slam` is a wrapper for the C++ class
****[mrpt::slam::CMetricMapBuilderRBPF](http://reference.mrpt.org/devel/classmrpt_1_1slam_1_1_c_metric_map_builder_r_b_p_f.html),
part of MRPT. Check out the documentation of this class for further
details.

This node has been designed to provide an interface similar to that of
[slam_gmapping](https://index.ros.org/p/slam_gmapping) for the
convenience of users who already know this package.

The coordinate frames are implemented according to the convention
[ROS REP 105: "Coordinate Frames for Mobile Platforms"](http://ros.org/reps/rep-0105.html).

## Usage

In order to use mrpt_rbpf_slam package it is necessary to install the
latest [MRPT](http://www.mrpt.org/) build and the
[mrpt_navigation](http://wiki.ros.org/mrpt_navigation) (see also the
[tutorial](http://wiki.ros.org/mrpt_navigation/Tutorials/Installing)).
Additionally, it is possible to install
[mvsim](http://wiki.ros.org/mvsim) simulator which allows to make
simulations with RBPF SLAM.

## Example

The following video demonstrates the work of the gridmap RBPF-based SLAM
using Rviz and MRPT GUI:

[Youtube](https://www.youtube.com/watch?v=KLMrDmq3G5E&feature)

To run the example of the gridmap RBPF SLAM in online mode, use the
following command:



```
roslaunch mrpt_rbpf_slam rbpf_slam.launch
```



To run the example of the gridmap RBPF SLAM from \*.rawlog file, use the
following command:



```
roslaunch mrpt_rbpf_slam rbpf_slam_rawlog.launch
```



another example is available by running



```
roslaunch mrpt_rbpf_slam rbpf_slam_rawlog.launch example:=2
```



Another possibility is to use the RBPF SLAM package for the range-only
SLAM. The following video demonstrates work of the range only RBPF-based
SLAM using Rviz and MRPT GUI:

[Youtube](https://www.youtube.com/watch?v=ydKxWQh6_qU)

The example usage of the range only RBPF SLAM in online mode:



```
roslaunch mrpt_rbpf_slam ro_slam.launch
```



The example usage of the range only RBPF SLAM from .rawlog file:



```
roslaunch mrpt_rbpf_slam ro_slam_rawlog.launch
```



The following video shows use of the mrpt_rbpf_slam package with mvsim
simulator:

[Youtube](https://www.youtube.com/watch?v=vEjsDGCdkjw)

The example usage of the mrpt_rbpf_slam with mvsim simulator:



```
roslaunch mrpt_rbpf_slam mvsim_slam.launch
```



The package was developed and tested as part of the Google Summer of
Code 2016 project. The following video shows the final tests carried out
with the robot:

[Youtube](https://www.youtube.com/watch?v=HfkRK3lwEgg)

## Nodes

### mrpt_rbpf_slam

The `mrpt_rbpf_slam` node takes in
[sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)
messages and builds a gridmap
([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html))
or range only map. The map can be retrieved via a ROS [topic](Topics).

#### Subscribed Topics

- `tf`
  ([tf/msg/tfMessage](https://docs.ros.org/en/rolling/p/tf/msg/tfMessage.html)):
  Transforms necessary to relate frames for laser, base, and odometry
  (see below)
- `scan`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  Laser scans to create the map from
- `beacon`
  ([mrpt_msgs/msg/ObservationRangeBeacon](https://docs.ros.org/en/rolling/p/mrpt_msgs/msg/ObservationRangeBeacon.html)):
  The beacons for range only map

#### Published Topics

- `particlecloud_beacons`
  ([geometry_msgs/msg/PoseArray](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseArray.html)):
  The particle cloud where each particle represents the possible
  location of the beacon
- `map`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Occupancy grid map topic
- `particlecloud`
  ([geometry_msgs/msg/PoseArray](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseArray.html)):
  Estimate of the robot's pose
- `beacon_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the beacons with ID

#### Parameters

- `~global_frame_id` (`string`, default: `"map"`): The frame
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
  file (if the rawlog file does not exist, the SLAM starts in the online
  mode)
- `~rawlog_play_delay` (`float`, default: `0.1`): Delay for replaying
  SLAM from the rawlog file

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

------------------------------------------------------------------------
