/*
 *  File:mrpt_ekf_slam_3d.h
 *  Author: Vladislav Tananaev
 *
 */


#ifndef MRPT_EKF_SLAM_2D_H
#define MRPT_EKF_SLAM_2D_H

#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/utils/CConfigFile.h>
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CRangeBearingKFSLAM2D.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/obs/CObservationBearingRange.h>
using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::opengl;
using namespace mrpt::system;
using namespace mrpt::math;
using namespace mrpt::poses;
using namespace mrpt::utils;
using namespace mrpt::obs;
using namespace std;

/**
 * @brief The EKFslam class provides EKF SLAM 2d from MRPT libraries. 
 *   
 */
class EKFslam{

public:
   /**
   * @brief constructor
   */
    EKFslam();
   /**
   * @brief destructor
   */
    ~EKFslam();
  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
    void read_iniFile(std::string ini_filename);
   /**
   * @brief calculate the actions from odometry model for current observation
   *
   * @param _sf  current observation
   * @param _odometry raw odometry
   */   
    void observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry);

protected:

    CRangeBearingKFSLAM2D mapping;///<EKF slam 2d class

    mrpt::system::TTimeStamp timeLastUpdate_;///< last update of the pose and map

    CActionCollectionPtr action;///< actions
	CSensoryFramePtr sf;///< observations

    mrpt::poses::CPose2D odomLastObservation_;  ///< last observation of odometry
    bool use_motion_model_default_options_; ///< used default odom_params
	CActionRobotMovement2D::TMotionModelOptions motion_model_default_options_; ///< used if there are is not odom
	CActionRobotMovement2D::TMotionModelOptions motion_model_options_;         ///< used with odom value motion noise

 



};


#endif /* MRPT_EKF_SLAM_2D_H */
