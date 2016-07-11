/*
 * File: mrpt_ekf_slam_3d_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_EKF_SLAM_3D_WRAPPER_H
#define MRPT_EKF_SLAM_3D_WRAPPER_H
#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <string>
#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d.h"
//add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>

class EKFslamWrapper : EKFslam{

public:
    EKFslamWrapper();
    ~EKFslamWrapper();

    void get_param();
    void init();
    bool rawlogPlay();
    bool is_file_exists(const std::string& name);
private:
    ros::NodeHandle n_;
    double rawlog_play_delay;///< delay of replay from rawlog file
    bool rawlog_play_;///< true if rawlog file exists

    std::string rawlog_filename;///< name of rawlog file
    std::string ini_filename;///< name of ini file
    std::string global_frame_id;///< /map frame
    std::string odom_frame_id; ///< /odom frame
    std::string base_frame_id; ///< robot frame
    //read rawlog file
    std::vector<std::pair<CActionCollection,CSensoryFrame>> data;///< vector of pairs of actions and obsrvations from rawlog file
    //Sensor source
    std::string sensor_source;///< 2D laser scans

    CTicTac	tictac;///<timer for SLAM performance evaluation
	float	t_exec;///<the time which take one SLAM update execution 
};


#endif /* MRPT_EKF_SLAM_3D_WRAPPER_H */ 


