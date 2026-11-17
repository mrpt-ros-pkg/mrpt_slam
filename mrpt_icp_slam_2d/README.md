# mrpt_icp_slam_2d

2D SLAM based on ICP scan matching of laser scans, building a point map and
an occupancy grid map.

The ROS 2 node `mrpt_icp_slam_2d` is a wrapper for the C++ class
[mrpt::slam::CMetricMapBuilderICP](https://docs.mrpt.org/reference/latest/class_mrpt_slam_CMetricMapBuilderICP.html),
part of [MRPT](https://github.com/MRPT/mrpt/). The ICP parameters and the
maps to build are configured in an MRPT INI file (see
`tutorial/icp_slam_demo.ini`).

Coordinate frames follow
[REP 105: "Coordinate Frames for Mobile Platforms"](https://www.ros.org/reps/rep-0105.html).

## Installation

Binary packages:

```bash
sudo apt install ros-$ROS_DISTRO-mrpt-icp-slam-2d
```

Or build from sources by cloning this repository into a colcon workspace.

## Demos

ICP SLAM offline, from the `.rawlog` dataset bundled with the package:

```bash
ros2 launch mrpt_icp_slam_2d icp_slam_rawlog.launch.py
```

ICP SLAM in the [mvsim](https://index.ros.org/p/mvsim) simulator (drive the
robot with the keyboard from the mvsim GUI):

```bash
ros2 launch mrpt_icp_slam_2d mvsim_icp_slam.launch.py
```

ICP SLAM online, from live (or bag-played) laser scans:

```bash
ros2 launch mrpt_icp_slam_2d icp_slam.launch.py
```

Its launch arguments are `ini_filename`, `global_frame_id`, `odom_frame_id`,
`base_frame_id`, `sensor_source` (default: `r1/front_laser/scan`),
`trajectory_update_rate`, `trajectory_publish_rate`, `use_sim_time`,
`node_namespace`, `launch_rviz`, and `rviz_config`.
`icp_slam_gui.launch.py` is equivalent, without RViz.

When the node exits, the built map is saved to the current directory as
`mrpt_icpslam_<date>_<time>.simplemap`.

## Node: mrpt_icp_slam_2d

### Subscribed topics

- `<sensor_source>`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  Laser scans. One subscription per topic in `sensor_source`.
- `/tf`, `/tf_static`: Transforms relating the sensor, base, and odometry
  frames (see below).

### Published topics

- `map`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Occupancy grid map, if one is defined in the INI file.
- `map_metadata`
  ([nav_msgs/msg/MapMetaData](https://docs.ros.org/en/rolling/p/nav_msgs/msg/MapMetaData.html)):
  Grid map metadata.
- `PointCloudMap`
  ([sensor_msgs/msg/PointCloud2](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/PointCloud2.html)):
  Point map, if one is defined in the INI file.
- `robot_pose`
  ([geometry_msgs/msg/PoseStamped](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseStamped.html)):
  Current robot pose estimate, in the global frame.
- `trajectory`
  ([nav_msgs/msg/Path](https://docs.ros.org/en/rolling/p/nav_msgs/msg/Path.html)):
  Estimated robot trajectory.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `ini_filename` | string | `""` | Path to the MRPT INI file with the ICP and map settings. |
| `rawlog_filename` | string | `""` | Path to a `.rawlog` file. If the file exists, SLAM runs offline from it; otherwise, the node runs online from ROS topics. |
| `rawlog_play_delay` | double | `0.1` | Delay [s] between steps while replaying a rawlog file. |
| `sensor_source` | string | `"scan"` | Comma- or space-separated list of laser scan topics. |
| `global_frame_id` | string | `"map"` | Frame attached to the map. |
| `odom_frame_id` | string | `"odom"` | Frame attached to the odometry system. |
| `base_frame_id` | string | `"base_link"` | Frame attached to the mobile base. |
| `trajectory_update_rate` | double | `10.0` | Rate [Hz] at which the trajectory is updated. |
| `trajectory_publish_rate` | double | `5.0` | Rate [Hz] at which `trajectory` is published. |

### Required tf transforms

- `<sensor frame>` → `base_link`: Relative pose of the laser sensor w.r.t.
  the robot. Usually fixed, published by
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher) or a
  `tf2_ros` `static_transform_publisher`.
- `odom` → `base_link`: Robot odometry, provided by the odometry system.

### Provided tf transforms

- `map` → `odom`: Current estimate of the robot pose within the map frame.
