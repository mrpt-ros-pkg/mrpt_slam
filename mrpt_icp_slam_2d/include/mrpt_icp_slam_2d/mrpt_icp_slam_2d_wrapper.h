/*
 * File: mrpt_icp_slam_2d_wrapper.h
 * Author: Vladislav Tananaev
 *
 */

#ifndef MRPT_ICP_SLAM_2D_WRAPPER_H
#define MRPT_ICP_SLAM_2D_WRAPPER_H

//MRPT libraries
#include <mrpt/slam/CMetricMapBuilderICP.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/opengl/CPlanarLaserScan.h>  // This class lives in the lib [mrpt-maps] and must be included by hand
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/poses/CPose3DPDF.h>

#include <iostream>     // std::cout
#include <fstream>      // std::ifstream
#include <string>

//add ros libraries
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
//add ros msgs
#include <nav_msgs/OccupancyGrid.h>
#include "nav_msgs/MapMetaData.h"
#include <std_msgs/String.h>
#include <std_msgs/Header.h>
#include <std_msgs/Int32.h>
#include <nav_msgs/GetMap.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <sensor_msgs/PointCloud.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>

//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/map.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/laser_scan.h>
#include <mrpt_bridge/time.h>
#include <mrpt_bridge/point_cloud.h>

using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::obs;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::poses;

class ICPslamWrapper{
public:
    ICPslamWrapper();
    ~ICPslamWrapper();
    void read_iniFile(std::string ini_filename);


  
    void get_param();
    void init();
    bool rawlogPlay();
    bool is_file_exists(const std::string& name);
    void laserCallback(const sensor_msgs::LaserScan &_msg);
    void publishTF();
    void publishMapPose();
    void updateSensorPose (std::string _frame_id);
private:
    CMetricMapBuilderICP mapBuilder;
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
    std::map<std::string, mrpt::poses::CPose3D> laser_poses_;///< laser scan poses with respect to the map

 
    //Subscribers
    std::vector<ros::Subscriber> sensorSub_;///< list of sensors topics

     CMultiMetricMap *metric_map_; ///<receive map after iteration of SLAM to metric map
     CPose3DPDFPtr curPDF;///<current robot pose
     ros::Publisher pub_map_, pub_metadata_, pub_pose_,pub_point_cloud_;///<publishers for map and pose particles

    tf::TransformListener listenerTF_;///<transform listener
     tf::TransformBroadcaster tf_broadcaster_;///<transform broadcaster


    CTicTac	tictac;///<timer for SLAM performance evaluation
	float	t_exec;///<the time which take one SLAM update execution 
    CSensoryFramePtr sf;///< observations
    mrpt::system::TTimeStamp timeLastUpdate_;///< last update of the pose and map

    ros::Time stamp;
};

#endif /* MRPT_ICP_SLAM_2D_WRAPPER_H */
