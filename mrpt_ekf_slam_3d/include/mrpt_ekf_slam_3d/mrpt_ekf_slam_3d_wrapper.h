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


////////////////////////
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
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//mrpt msgs
#include "mrpt_msgs/ObservationRangeBeacon.h"
//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/beacon.h>
#include <mrpt_bridge/time.h>
////////////////////////////
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/stock_objects.h>

//////////////
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
	CPose3DQuatPDFGaussian	  robotPose_;
	//CPose3D                   robotPose_;
	//CPoint3D                  lm_;
    //std::vector<typename IMPL::landmark_point_t>	 LMs_;
   std::vector<CPoint3D>	 LMs_;
	std::map<unsigned int,CLandmark::TLandmarkID>    LM_IDs_;
	CMatrixDouble  fullCov_;
	CVectorDouble  fullState_;
    ros:: Publisher  pub_Particles_Beacons_;
};


#endif /* MRPT_EKF_SLAM_3D_WRAPPER_H */ 


