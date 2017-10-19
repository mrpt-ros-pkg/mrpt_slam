/*
 * File: mrpt_rbpf_slam_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_RBPF_SLAM_WRAPPER_H
#define MRPT_RBPF_SLAM_WRAPPER_H

#include <iostream>  // std::cout
#include <fstream>   // std::ifstream
#include <string>
#include "mrpt_rbpf_slam/mrpt_rbpf_slam.h"

// add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
// add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
// mrpt msgs
#include "mrpt_msgs/ObservationRangeBeacon.h"
// mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/beacon.h>
#include <mrpt_bridge/time.h>

#if MRPT_VERSION >= 0x130
#include <mrpt/obs/CObservationBeaconRanges.h>
using namespace mrpt::obs;
#else
#include <mrpt/slam/CObservationBeaconRanges.h>
using namespace mrpt::slam;
#endif

/**
 * @brief The PFslamWrapper class provides  the ROS wrapper for Rao-Blackwellized Particle filter SLAM from MRPT
 *libraries.
 *
 */
class PFslamWrapper : PFslam
{
public:
  /**
  * @brief constructor
  */
  PFslamWrapper();
  /**
 * @brief destructor
 */
  ~PFslamWrapper();
  /**
 * @brief read the parameters from launch file
 */
  void get_param();
  /**
  * @brief initialize publishers subscribers and RBPF slam
  */
  void init();
  /**
  * @brief play rawlog file
  *
  * @return true if rawlog file exists and played
  */
  bool rawlogPlay();
  /**
  * @brief publish beacon or grid map and robot pose
  *
  */
  void publishMapPose();

  /**
  * @brief check the existance of the file
  *
  * @return true if file exists
  */
  bool is_file_exists(const std::string& name);

  /**
  * @brief callback function for the beacons
  *
  * Given the range only observation wait for odometry,
  * create the pair of action and observation,
  * implement one SLAM update,
  * publish map and pose.
  *
  * @param _msg  the beacon message
  */
  void callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& _msg);

  /**
  * @brief callback function for the laser scans
  *
  * Given the laser scans  wait for odometry,
  * create the pair of action and observation,
  * implement one SLAM update,
  * publish map and pose.
  *
  * @param _msg  the laser scan message
  */
  void laserCallback(const sensor_msgs::LaserScan& _msg);

  /**
   * @brief wait for transfor between odometry frame and the robot frame
   *
   * @param des position of the robot with respect to odometry frame
   * @param target_frame the odometry tf frame
   * @param source_frame the robot tf frame
   * @param time timestamp of the observation for which we want to retrieve the position of the robot
   * @param timeout timeout for odometry waiting
   * @param polling_sleep_duration timeout for transform wait
   *
   * @return true if there is transform from odometry to the robot
   */
  bool waitForTransform(mrpt::poses::CPose3D& des, const std::string& target_frame, const std::string& source_frame,
                        const ros::Time& time, const ros::Duration& timeout,
                        const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

  /**
  * @brief  get  the odometry for received observation
  *
  * @param _odometry odometry for received observation
  * @param _msg_header timestamp of the observation
  */
  void odometryForCallback(CObservationOdometry::Ptr& _odometry, const std_msgs::Header& _msg_header);

  /**
  * @brief  update the pose of the sensor with respect to the robot
  *
  *@param frame_id the frame of the sensors
  */
  void updateSensorPose(std::string frame_id);

  /**
  * @brief  publis tf tree
  *
  */
  void publishTF();
  /**
  * @brief  correct visualization for ro slam (under development)
  *
  */
  void vizBeacons();

private:
  ros::NodeHandle n_;        ///< Node Handle
  double rawlog_play_delay;  ///< delay of replay from rawlog file
  bool rawlog_play_;         ///< true if rawlog file exists

  std::string rawlog_filename;  ///< name of rawlog file
  std::string ini_filename;     ///< name of ini file
  std::string global_frame_id;  ///< /map frame
  std::string odom_frame_id;    ///< /odom frame
  std::string base_frame_id;    ///< robot frame

  // Sensor source
  std::string sensor_source;  ///< 2D laser scans

  std::map<std::string, mrpt::poses::CPose3D> laser_poses_;   ///< laser scan poses with respect to the map
  std::map<std::string, mrpt::poses::CPose3D> beacon_poses_;  ///< beacon poses with respect to the map

  // Subscribers
  std::vector<ros::Subscriber> sensorSub_;  ///< list of sensors topics

  // read rawlog file
  std::vector<std::pair<CActionCollection, CSensoryFrame>> data;  ///< vector of pairs of actions and obsrvations from
                                                                  ///rawlog file

  std::vector<mrpt::opengl::CEllipsoid::Ptr> viz_beacons;

  ros::Publisher pub_map_, pub_metadata_, pub_Particles_, pub_Particles_Beacons_,
      beacon_viz_pub_;  ///<publishers for map and pose particles

  tf::TransformListener listenerTF_;         ///<transform listener
  tf::TransformBroadcaster tf_broadcaster_;  ///<transform broadcaster

  CTicTac tictac;  ///<timer for SLAM performance evaluation
  float t_exec;    ///<the time which take one SLAM update execution
};

#endif /*MRPT_RBPF_SLAM_WRAPPER_H*/
