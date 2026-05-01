[![CI Build colcon](https://github.com/mrpt-ros-pkg/mrpt_slam/actions/workflows/build-ros.yml/badge.svg)](https://github.com/mrpt-ros-pkg/mrpt_slam/actions/workflows/build-ros.yml)

| Distro | Build dev | Stable sync |
| --- | --- | --- |
| ROS 2 Humble (u22.04) | [![Build Status](https://build.ros2.org/job/Hdev__mrpt_slam__ubuntu_jammy_amd64/badge/icon)](https://build.ros2.org/job/Hdev__mrpt_slam__ubuntu_jammy_amd64/) | [![Version](https://img.shields.io/ros/v/humble/mrpt_slam)](https://index.ros.org/search/?term=mrpt_slam) |
| ROS 2 Jazzy (u24.04) | [![Build Status](https://build.ros2.org/job/Jdev__mrpt_slam__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Jdev__mrpt_slam__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/jazzy/mrpt_slam)](https://index.ros.org/search/?term=mrpt_slam) |
| ROS 2 Kilted (u24.04) | [![Build Status](https://build.ros2.org/job/Kdev__mrpt_slam__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Kdev__mrpt_slam__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/kilted/mrpt_slam)](https://index.ros.org/search/?term=mrpt_slam) |
| ROS 2 Rolling (u24.04) | [![Build Status](https://build.ros2.org/job/Rdev__mrpt_slam__ubuntu_noble_amd64/badge/icon)](https://build.ros2.org/job/Rdev__mrpt_slam__ubuntu_noble_amd64/) | [![Version](https://img.shields.io/ros/v/rolling/mrpt_slam)](https://index.ros.org/search/?term=mrpt_slam) |

| EOL Distro | Last version |
| --- | --- |
| ROS 1 Noetic (u20.04) | [![Version](https://img.shields.io/ros/v/noetic/mrpt_slam)](https://index.ros.org/search/?term=mrpt_slam) |

<img align="center" src="https://mrpt.github.io/imgs/mrpt_slam_ros_pkg_demo_video_s90.gif">


MRPT-based SLAM packages
-------------------------

ROS 2 wrappers for SLAM algorithms in the [Mobile Robot Programming Toolkit (MRPT)](https://github.com/MRPT/mrpt/).
Refer to https://wiki.ros.org/mrpt_slam for further documentation.

Active branches:
  * `ros2`: Main development branch, for ROS 2 distributions.
  * `ros1`: Legacy branch for ROS 1 (no further development).

Packages
---------
* [mrpt_ekf_slam_2d](mrpt_ekf_slam_2d): EKF-based SLAM with range-bearing sensors, 2D robot pose, and 2D landmarks.
* [mrpt_ekf_slam_3d](mrpt_ekf_slam_3d): EKF-based SLAM for 3D environments with a full 6D robot pose and 3D landmarks.
* [mrpt_icp_slam_2d](mrpt_icp_slam_2d): ICP scan-matching SLAM for 2D occupancy grid mapping.
* [mrpt_rbpf_slam](mrpt_rbpf_slam): Rao-Blackwellized Particle Filter SLAM (similar to gmapping, with more sensor support).
* [mrpt_graphslam_2d](mrpt_graphslam_2d): Graph-SLAM with single-robot and (experimental) multi-robot support.

Individual package build status
---------------------------------

| Package | ROS 2 Humble <br/> BinBuild | ROS 2 Jazzy <br/> BinBuild | ROS 2 Kilted <br/> BinBuild | ROS 2 Rolling <br/> BinBuild |
| --- | --- | --- | --- | --- |
| mrpt_ekf_slam_2d | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_ekf_slam_2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_ekf_slam_2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_ekf_slam_2d__ubuntu_noble_amd64__binary/) |
| mrpt_ekf_slam_3d | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_ekf_slam_3d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_ekf_slam_3d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_ekf_slam_3d__ubuntu_noble_amd64__binary/) |
| mrpt_icp_slam_2d | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_icp_slam_2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_icp_slam_2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_icp_slam_2d__ubuntu_noble_amd64__binary/) |
| mrpt_rbpf_slam | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_rbpf_slam__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_rbpf_slam__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_rbpf_slam__ubuntu_noble_amd64__binary/) |
| mrpt_graphslam_2d | [![Build Status](https://build.ros2.org/job/Hbin_uJ64__mrpt_graphslam_2d__ubuntu_jammy_amd64__binary/badge/icon)](https://build.ros2.org/job/Hbin_uJ64__mrpt_graphslam_2d__ubuntu_jammy_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Jbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Jbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Kbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Kbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/) | [![Build Status](https://build.ros2.org/job/Rbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/badge/icon)](https://build.ros2.org/job/Rbin_uN64__mrpt_graphslam_2d__ubuntu_noble_amd64__binary/) |


Contributing
----------------------------------
* Code formatting: We use clang-format. Invoke it from the root directory as:

      bash clang-formatter.sh

**Contributors**

<a href="https://github.com/mrpt-ros-pkg/mrpt_slam/graphs/contributors">
  <img src="https://contrib.rocks/image?repo=mrpt-ros-pkg/mrpt_slam" />
</a>

