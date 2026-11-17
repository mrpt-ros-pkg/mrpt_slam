# mrpt_ekf_slam_2d

EKF-based 2D SLAM for a robot pose (x, y, phi) and 2D point landmarks,
using range-bearing observations.

The ROS 2 node `mrpt_ekf_slam_2d` is a wrapper for the C++ class
[mrpt::slam::CRangeBearingKFSLAM2D](https://docs.mrpt.org/reference/latest/class_mrpt_slam_CRangeBearingKFSLAM2D.html),
part of [MRPT](https://github.com/MRPT/mrpt/). Check out the documentation
of that class for further details on the algorithm and its INI options.

Coordinate frames follow
[REP 105: "Coordinate Frames for Mobile Platforms"](https://www.ros.org/reps/rep-0105.html).

## Installation

Binary packages:

```bash
sudo apt install ros-$ROS_DISTRO-mrpt-ekf-slam-2d
```

Or build from sources by cloning this repository into a colcon workspace.

## Demos

The following video shows EKF 2D SLAM in RViz and the MRPT GUI:
[YouTube](https://www.youtube.com/watch?v=tGMmi6Y6bL8)

Run EKF SLAM offline, from the `.rawlog` dataset bundled with the package:

```bash
ros2 launch mrpt_ekf_slam_2d ekf_slam_2d_rawlog.launch.py
```

Run EKF SLAM online, subscribing to live (or bag-played) landmark
observations:

```bash
ros2 launch mrpt_ekf_slam_2d ekf_slam_2d.launch.py
```

Launch arguments of `ekf_slam_2d.launch.py`: `ini_filename`, `ellipse_scale`,
`global_frame_id`, `odom_frame_id`, `base_frame_id`, `sensor_source`
(default: `landmark`), `use_sim_time`, `launch_rviz`, and `rviz_config`.

## Node: mrpt_ekf_slam_2d

### Subscribed topics

- `<sensor_source>`
  ([mrpt_msgs/msg/ObservationRangeBearing](https://index.ros.org/p/mrpt_msgs)):
  Range-bearing landmark observations. One subscription per topic in
  `sensor_source`.
- `/tf`, `/tf_static`: Transforms relating the sensor, base, and odometry
  frames (see below).

### Published topics

- `/state_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the current EKF state (robot pose and landmarks, with
  covariance ellipses).
- `/data_association_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the landmarks currently observed by the robot.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `ini_filename` | string | `""` | Path to the MRPT INI file with the SLAM settings. See `tutorial/kf-slam_demo_2d.ini`. |
| `rawlog_filename` | string | `""` | Path to a `.rawlog` file. If the file exists, SLAM runs offline from it; otherwise, the node runs online from ROS topics. |
| `rawlog_play_delay` | double | `0.1` | Delay [s] between steps while replaying a rawlog file. |
| `sensor_source` | string | `"scan"` | Comma- or space-separated list of input topics. Only topics whose name contains `landmark` are used, so this must be set (the launch files use `landmark`). |
| `global_frame_id` | string | `"map"` | Frame attached to the map. |
| `odom_frame_id` | string | `"odom"` | Frame attached to the odometry system. |
| `base_frame_id` | string | `"base_link"` | Frame attached to the mobile base. |
| `ellipse_scale` | double | `1.0` | Scale factor for the covariance ellipses in the visualization. |

### Required tf transforms

- `<sensor frame>` → `base_link`: Relative pose of the sensor w.r.t. the
  robot. Usually fixed, published by
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher) or a
  `tf2_ros` `static_transform_publisher`.
- `odom` → `base_link`: Robot odometry, provided by the odometry system
  (e.g., the driver of the mobile base).

### Provided tf transforms

- `map` → `odom`: Current estimate of the robot pose within the map frame.
