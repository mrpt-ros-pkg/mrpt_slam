# mrpt_rbpf_slam

Rao-Blackwellized Particle Filter (RBPF) SLAM. The interface is similar to
[slam_gmapping](https://index.ros.org/p/slam_gmapping), but this package
supports different particle filter algorithms, range-only (beacon) SLAM, and
several simultaneous grid maps.

Supported particle filter algorithms:

- Sequential Importance Resampling (SIR) with the "standard" proposal
  distribution (`pfStandardProposal`).
- Auxiliary Particle Filter (APF) with the "standard" proposal distribution
  (`pfAuxiliaryPFStandard`).
- Optimal proposal distribution (`pfOptimalProposal`).
- Approximate optimal sampling: a rejection sampling-based approximation to
  the optimal proposal, when no closed-form expression is available
  (`pfAuxiliaryPFOptimal`).

The ROS 2 node `mrpt_rbpf_slam` is a wrapper for the C++ class
[mrpt::slam::CMetricMapBuilderRBPF](https://docs.mrpt.org/reference/latest/class_mrpt_slam_CMetricMapBuilderRBPF.html),
part of [MRPT](https://github.com/MRPT/mrpt/). The algorithm, the particle
filter, and the map types are configured in an MRPT INI file (see the
`tutorial/*.ini` examples).

Coordinate frames follow
[REP 105: "Coordinate Frames for Mobile Platforms"](https://www.ros.org/reps/rep-0105.html).

## Installation

Binary packages:

```bash
sudo apt install ros-$ROS_DISTRO-mrpt-rbpf-slam
```

Or build from sources by cloning this repository into a colcon workspace.
Optionally, install the [mvsim](https://index.ros.org/p/mvsim) simulator to
run the simulation demo.

## Demos

Grid map RBPF SLAM in RViz and the MRPT GUI:
[YouTube](https://www.youtube.com/watch?v=KLMrDmq3G5E)

Grid map RBPF SLAM offline, from the `.rawlog` datasets bundled with the
package:

```bash
ros2 launch mrpt_rbpf_slam rbpf_slam_rawlog.launch.py
# Another dataset:
ros2 launch mrpt_rbpf_slam rbpf_slam_rawlog.launch.py example:=2
```

Grid map RBPF SLAM online, from live (or bag-played) laser scans:

```bash
ros2 launch mrpt_rbpf_slam rbpf_slam.launch.py
```

Its launch arguments are `config_file` (default: `config/default.yaml`),
`ini_filename`, `global_frame_id`, `odom_frame_id`, `base_frame_id`,
`sensor_source`, `use_sim_time`, `node_namespace`, `launch_rviz`, and
`rviz_config`. A TurtleBot3 example is available in
`rbpf_slam_turtlebot3.launch.py`.

Range-only SLAM with radio beacons:
[YouTube](https://www.youtube.com/watch?v=ydKxWQh6_qU)

```bash
ros2 launch mrpt_rbpf_slam ro_slam.launch.py
```

RBPF SLAM with the mvsim simulator:
[YouTube](https://www.youtube.com/watch?v=vEjsDGCdkjw)

```bash
ros2 launch mrpt_rbpf_slam mvsim_slam.launch.py
```

This package was originally developed and tested as part of a Google Summer
of Code 2016 project. Final tests with a real robot:
[YouTube](https://www.youtube.com/watch?v=HfkRK3lwEgg)

## Node: mrpt_rbpf_slam

### Subscribed topics

- `<sensor_source>`
  ([sensor_msgs/msg/LaserScan](https://docs.ros.org/en/rolling/p/sensor_msgs/msg/LaserScan.html)):
  Laser scans to build the map from. One subscription per topic in
  `sensor_source`. Range-only (beacon) observations are currently only
  supported when replaying a rawlog file.
- `/tf`, `/tf_static`: Transforms relating the sensor, base, and odometry
  frames (see below).

### Published topics

- `map`
  ([nav_msgs/msg/OccupancyGrid](https://docs.ros.org/en/rolling/p/nav_msgs/msg/OccupancyGrid.html)):
  Occupancy grid map.
- `map_metadata`
  ([nav_msgs/msg/MapMetaData](https://docs.ros.org/en/rolling/p/nav_msgs/msg/MapMetaData.html)):
  Grid map metadata.
- `particlecloud`
  ([geometry_msgs/msg/PoseArray](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseArray.html)):
  Robot pose particles.
- `particlecloud_beacons`
  ([geometry_msgs/msg/PoseArray](https://docs.ros.org/en/rolling/p/geometry_msgs/msg/PoseArray.html)):
  Particles representing the possible locations of beacons (range-only SLAM).
- `/beacons_viz`
  ([visualization_msgs/msg/MarkerArray](https://docs.ros.org/en/rolling/p/visualization_msgs/msg/MarkerArray.html)):
  Visualization of the beacons, with their IDs.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `ini_filename` | string | `""` | Path to the MRPT INI file with the SLAM settings (see `tutorial/`). |
| `rawlog_filename` | string | `""` | Path to a `.rawlog` file. If the file exists, SLAM runs offline from it; otherwise, the node runs online from ROS topics. |
| `rawlog_play_delay` | double | `0.1` | Delay [s] between steps while replaying a rawlog file. |
| `sensor_source` | string | `"scan"` | Comma- or space-separated list of laser scan topics. |
| `global_frame_id` | string | `"map"` | Frame attached to the map. |
| `odom_frame_id` | string | `"odom"` | Frame attached to the odometry system. |
| `base_frame_id` | string | `"base_link"` | Frame attached to the mobile base. |
| `update_sensor_pose` | bool | `true` | Whether to look up the sensor pose on `/tf` for each observation. |
| `simplemap_save_folder` | string | `"/tmp/"` | Folder where the resulting `.simplemap` file is saved. |
| `motion_model.type` | string | `"thrun"` | Odometry motion model: `thrun` or `gaussian`. |
| `motion_model.thrun_motion_model_options.*` | double/int | see `config/default.yaml` | Parameters of the Thrun motion model: `particle_count`, `alfa1_rot_rot`, `alfa2_rot_trans`, `alfa3_trans_trans`, `alfa4_trans_rot`, `additional_std_XY`, `additional_std_phi`. |
| `motion_model.gaussian_motion_model_options.*` | double | see `config/default.yaml` | Parameters of the Gaussian motion model: `a1`, `a2`, `a3`, `a4`, `minStdXY`, `minStdPHI`. |
| `mrpt_visualization_options.show_window` | bool | `true` | Show the MRPT 3D GUI window. |
| `mrpt_visualization_options.width`, `.height` | int | `600`, `500` | Size of the MRPT GUI window. |
| `mrpt_visualization_options.window_update_delay` | int | `1` | Delay [ms] between GUI window updates. |
| `mrpt_visualization_options.camera_follow_robot` | bool | `false` | Whether the GUI camera follows the robot. |

See [config/default.yaml](config/default.yaml) for a commented example with
all the parameters above.

### Required tf transforms

- `<sensor frame>` → `base_link`: Relative pose of the laser sensor w.r.t.
  the robot. Usually fixed, published by
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher) or a
  `tf2_ros` `static_transform_publisher`.
- `odom` → `base_link`: Robot odometry, provided by the odometry system
  (e.g., the driver of the mobile base).

### Provided tf transforms

- `map` → `odom`: Current estimate of the robot pose within the map frame.
