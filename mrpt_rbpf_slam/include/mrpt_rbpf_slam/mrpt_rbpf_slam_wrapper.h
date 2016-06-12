/*
 * File: mrpt_rbpf_slam_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_RBPF_SLAM_WRAPPER_H
#define MRPT_RBPF_SLAM_WRAPPER_H

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream

#include "mrpt_rbpf_slam/mrpt_rbpf_slam.h"




//add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>

//mrpt msgs
#include "mrpt_msgs/ObservationRangeBeacon.h"
//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/beacon.h>
#include <mrpt_bridge/time.h>

class PFslamWrapper:PFslam{
public:
    int counter;
    int counter2;
    PFslamWrapper();
    ~PFslamWrapper();
    //the function read parameters from launch file
    void get_param();
    void init();
    void rawlogPlay();
    void publishMapPose();

    void loop();

    bool is_file_exists(const std::string& name);

    //callback function
void callbackBeacon (const mrpt_msgs::ObservationRangeBeacon &_msg) ;
    void laserCallback(const sensor_msgs::LaserScan & _msg);
    void callbackInitialpose (const geometry_msgs::PoseWithCovarianceStamped& _msg);
    
    bool waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01));
   void odometryForCallback (CObservationOdometryPtr  &_odometry, const std_msgs::Header &_msg_header);
    //update the laser poses with respect to the map 
    void updateSensorPose (std::string frame_id);
    void publishTF(); 
private:
    ros::NodeHandle n_;
    double rawlog_play_delay;
    bool rawlog_play_;
    //get_param() parameters
    std::string rawlog_filename;
    std::string ini_filename;
    std::string global_frame_id;// /map
    std::string odom_frame_id; // /odom
    std::string base_frame_id; //base_frame

    //Sensor source
    std::string sensor_source;//2D laser scans
 
    std::map<std::string, mrpt::poses::CPose3D> laser_poses_;//laser scan poses with respect to the map
    std::map<std::string, mrpt::poses::CPose3D> beacon_poses_;
 
    //Subscribers
    std::vector<ros::Subscriber> sensorSub_;

    ros::Subscriber subInitPose_;
 
 mrpt::poses::CPosePDFGaussian initialPose_;  /// initilial posed used in initializeFilter()

    //read rawlog file
    std::vector<std::pair<CActionCollection,CSensoryFrame>> data;

     //receive map after iteration of SLAM to metric map
     CMultiMetricMap *metric_map_;
    //received pose of robo
     CPose3DPDFParticles   curPDF;//current robot pose
     mrpt::poses::CPose3DPDFParticles  robotPoseEstimation;
    //publishers for map and pose particles
    ros::Publisher pub_map_, pub_metadata_,  pub_Particles_,pub_Particles_Beacons_;
    
    tf::TransformListener listenerTF_;
     tf::TransformBroadcaster tf_broadcaster_;

};


#endif /*MRPT_RBPF_SLAM_WRAPPER_H*/
