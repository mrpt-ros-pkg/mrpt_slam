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
//add ros msgs
#include <std_msgs/String.h>
#include <std_msgs/Int32.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <visualization_msgs/MarkerArray.h>
#include <visualization_msgs/Marker.h>
//mrpt bridge libs
#include <mrpt_bridge/pose.h>
#include <mrpt_bridge/landmark.h>
#include <mrpt_bridge/mrpt_log_macros.h>
#include <mrpt_bridge/time.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/utils/CConfigFile.h>
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
#include <mrpt_msgs/ObservationRangeBearing.h>
class EKFslamWrapper : EKFslam{

public:
    EKFslamWrapper();
    ~EKFslamWrapper();

    void get_param();
    void init();
    bool rawlogPlay();
    bool is_file_exists(const std::string& name);
    void viz_state();
    void odometryForCallback (CObservationOdometryPtr  &_odometry, const std_msgs::Header &_msg_header);
    void landmarkCallback(const mrpt_msgs::ObservationRangeBearing &_msg);
    void updateSensorPose (std::string _frame_id);
    bool waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration = ros::Duration(0.01));

 void publishTF(); 


private:
    ros::NodeHandle n_;
    double rawlog_play_delay;///< delay of replay from rawlog file
    bool rawlog_play_;///< true if rawlog file exists
    //Subscribers
    std::vector<ros::Subscriber> sensorSub_;///< list of sensors topics
    std::string rawlog_filename;///< name of rawlog file
    std::string ini_filename;///< name of ini file
    std::string global_frame_id;///< /map frame
    std::string odom_frame_id; ///< /odom frame
    std::string base_frame_id; ///< robot frame

    //Sensor source
    std::string sensor_source;///< 2D laser scans
 
    std::map<std::string, mrpt::poses::CPose3D> landmark_poses_;///< landmark poses with respect to the map

   


    CTicTac	tictac;///<timer for SLAM performance evaluation
	float	t_exec;///<the time which take one SLAM update execution 
    
	CPose3DQuatPDFGaussian	  robotPose_;
    std::vector<mrpt::math::TPoint3D> 	 LMs_;
	std::map<unsigned int,CLandmark::TLandmarkID>    LM_IDs_;
	CMatrixDouble  fullCov_;
	CVectorDouble  fullState_;
    ros:: Publisher  pub_Particles_Beacons_, state_viz_pub_;
    tf::TransformListener listenerTF_;///<transform listener
    tf::TransformBroadcaster tf_broadcaster_;///<transform broadcaster


};


#endif /* MRPT_EKF_SLAM_3D_WRAPPER_H */ 


