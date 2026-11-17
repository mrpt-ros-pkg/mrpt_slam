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

The ROS node `mrpt_rbpf_slam` is a wrapper for the C++ class
[mrpt::slam::CMetricMapBuilderRBPF](https://docs.mrpt.org/reference/latest/class_mrpt_slam_CMetricMapBuilderRBPF.html),
part of [MRPT](https://github.com/MRPT/mrpt/). The algorithm, the particle
filter, and the map types are configured in an MRPT INI file (see the
`tutorial/*.ini` examples).

Coordinate frames follow
[REP 105: "Coordinate Frames for Mobile Platforms"](https://www.ros.org/reps/rep-0105.html).

## Installation

Binary packages:

```bash
sudo apt install ros-noetic-mrpt-rbpf-slam
```

Or build from sources by cloning this repository into a catkin workspace.
Optionally, install the [mvsim](https://index.ros.org/p/mvsim) simulator to
run the simulation demo.

## Demos

Grid map RBPF SLAM in RViz and the MRPT GUI:
[YouTube](https://www.youtube.com/watch?v=KLMrDmq3G5E)

Grid map RBPF SLAM offline, from the `.rawlog` datasets bundled with the
package:

```bash
roslaunch mrpt_rbpf_slam rbpf_slam_rawlog.launch
# Another dataset:
roslaunch mrpt_rbpf_slam rbpf_slam_rawlog.launch example:=2
```

Grid map RBPF SLAM online, subscribing to laser scans from a demo dataset
played back by the `mrpt_rawlog` package:

```bash
roslaunch mrpt_rbpf_slam rbpf_slam.launch
```

A TurtleBot3 example is available in `rbpf_slam_turtlebot3.launch`.

Range-only SLAM with radio beacons:
[YouTube](https://www.youtube.com/watch?v=ydKxWQh6_qU)

```bash
roslaunch mrpt_rbpf_slam ro_slam_rawlog.launch
```

RBPF SLAM with the mvsim simulator:
[YouTube](https://www.youtube.com/watch?v=vEjsDGCdkjw)

```bash
roslaunch mrpt_rbpf_slam mvsim_slam.launch
```

This package was originally developed and tested as part of a Google Summer
of Code 2016 project. Final tests with a real robot:
[YouTube](https://www.youtube.com/watch?v=HfkRK3lwEgg)

## Node: mrpt_rbpf_slam

### Subscribed topics

- `<sensor_source>`
  ([sensor_msgs/LaserScan](http://docs.ros.org/en/noetic/api/sensor_msgs/html/msg/LaserScan.html)):
  Laser scans to build the map from. One subscription per topic in
  `sensor_source`. Range-only (beacon) observations are currently only
  supported when replaying a rawlog file.
- `/tf`, `/tf_static`: Transforms relating the sensor, base, and odometry
  frames (see below).

### Published topics

- `map`
  ([nav_msgs/OccupancyGrid](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/OccupancyGrid.html)):
  Occupancy grid map.
- `map_metadata`
  ([nav_msgs/MapMetaData](http://docs.ros.org/en/noetic/api/nav_msgs/html/msg/MapMetaData.html)):
  Grid map metadata.
- `particlecloud`
  ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)):
  Robot pose particles.
- `particlecloud_beacons`
  ([geometry_msgs/PoseArray](http://docs.ros.org/en/noetic/api/geometry_msgs/html/msg/PoseArray.html)):
  Particles representing the possible locations of beacons (range-only SLAM).
- `/beacons_viz`
  ([visualization_msgs/MarkerArray](http://docs.ros.org/en/noetic/api/visualization_msgs/html/msg/MarkerArray.html)):
  Visualization of the beacons, with their IDs.

### Parameters

| Name | Type | Default | Description |
| --- | --- | --- | --- |
| `update_loop_frequency` | double | `100.0` | Rate [Hz] of the node main loop. |
| `ini_filename` | string | `""` | Path to the MRPT INI file with the SLAM settings (see `tutorial/`). |
| `rawlog_filename` | string | `""` | Path to a `.rawlog` file. If the file exists, SLAM runs offline from it; otherwise, the node runs online from ROS topics. |
| `rawlog_play_delay` | double | `0.1` | Delay [s] between steps while replaying a rawlog file. |
| `sensor_source` | string | `"scan"` | Comma- or space-separated list of laser scan topics. |
| `global_frame_id` | string | `"map"` | Frame attached to the map. |
| `odom_frame_id` | string | `"odom"` | Frame attached to the odometry system. |
| `base_frame_id` | string | `"base_link"` | Frame attached to the mobile base. |
| `update_sensor_pose` | bool | `true` | Whether to look up the sensor pose on `/tf` for each observation. |
| `simplemap_save_folder` | string | (required) | Folder where the resulting `.simplemap` file is saved. |
| `motion_model.type` | string | (required) | Odometry motion model: `thrun` or `gaussian`. |
| `motion_model.thrun_motion_model_options.*` | double/int | (required) | Parameters of the Thrun motion model: `particle_count`, `alfa1_rot_rot`, `alfa2_rot_trans`, `alfa3_trans_trans`, `alfa4_trans_rot`, `additional_std_XY`, `additional_std_phi`. |
| `motion_model.gaussian_motion_model_options.*` | double | (required) | Parameters of the Gaussian motion model: `a1`, `a2`, `a3`, `a4`, `minStdXY`, `minStdPHI`. |
| `mrpt_visualization_options.show_window` | bool | (required) | Show the MRPT 3D GUI window. |
| `mrpt_visualization_options.width`, `.height` | int | (required) | Size of the MRPT GUI window. |
| `mrpt_visualization_options.window_update_delay` | int | (required) | Delay [ms] between GUI window updates. |
| `mrpt_visualization_options.camera_follow_robot` | bool | (required) | Whether the GUI camera follows the robot. |

Parameters marked as required have no built-in default: load
[config/default.yaml](config/default.yaml) with `<rosparam>`, as the
provided launch files do. `particle_count` defaults to `100`.

### Required tf transforms

- `<sensor frame>` → `base_link`: Relative pose of the laser sensor w.r.t.
  the robot. Usually fixed, published by
  [robot_state_publisher](https://index.ros.org/p/robot_state_publisher) or a
  `tf` `static_transform_publisher`.
- `odom` → `base_link`: Robot odometry, provided by the odometry system
  (e.g., the driver of the mobile base).

### Provided tf transforms

- `map` → `odom`: Current estimate of the robot pose within the map frame.
