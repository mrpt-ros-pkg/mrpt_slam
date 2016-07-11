/*
 * File: mrpt_icp_slam_2d_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_ICP_SLAM_2D_WRAPPER_H
#define MRPT_ICP_SLAM_2D_WRAPPER_H
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <string>
#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d.h"
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
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>

class ICPslamWrapper: ICPslam{
public:
    ICPslamWrapper();
    ~ICPslamWrapper();

    void get_param();
    void init();
    bool rawlogPlay();
    bool is_file_exists(const std::string& name);



private:
    ros::NodeHandle n_;///< Node Handle
    double rawlog_play_delay;///< delay of replay from rawlog file
    bool rawlog_play_;///< true if rawlog file exists

    std::string rawlog_filename;///< name of rawlog file
    std::string ini_filename;///< name of ini file
    std::string global_frame_id;///< /map frame
    std::string odom_frame_id; ///< /odom frame
    std::string base_frame_id; ///< robot frame

    //Sensor source
    std::string sensor_source;///< 2D laser scans



     CMultiMetricMap *metric_map_; ///<receive map after iteration of SLAM to metric map
     CPose3DPDFPtr curPDF;///<current robot pose
     ros::Publisher pub_map_, pub_metadata_, pub_pose_;///<publishers for map and pose particles

    tf::TransformListener listenerTF_;///<transform listener
     tf::TransformBroadcaster tf_broadcaster_;///<transform broadcaster


    CTicTac	tictac;///<timer for SLAM performance evaluation
	float	t_exec;///<the time which take one SLAM update execution 
};

#endif /* MRPT_ICP_SLAM_2D_WRAPPER_H */
