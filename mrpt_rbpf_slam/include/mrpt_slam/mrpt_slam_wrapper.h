/*
 * File: mrpt_slam_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_SLAM_WRAPPER_H
#define MRPT_SLAM_WRAPPER_H


#include "mrpt_slam/mrpt_slam.h"

//add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>

//add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>

//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>


class PFslamWrapper:PFslam{
public:
    PFslamWrapper();
    ~PFslamWrapper();
    //the function read parameters from launch file
    void get_param();
    void init();
    void publishMap();

private:
    ros::NodeHandle n_;

    //get_param() parameters
    std::string rawlog_filename;
    std::string ini_filename;
    std::string global_frame_id;// /map
    std::string odom_frame_id; // /odom
    std::string base_frame_id; //base_frame
    std::string sensor_source; 

    //read rawlog file
    std::vector<std::pair<CActionCollection,CSensoryFrame>> data;

     //receive map after iteration of SLAM to metric map
     CMultiMetricMap *metric_map_;
      
    //publishers for map and pose particles
    ros::Publisher pub_map_, pub_metadata_,  pub_Particles_;
    

};


#endif /*MRPT_SLAM_WRAPPER_H*/
