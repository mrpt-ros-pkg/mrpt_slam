/*
 *  File:mrpt_ekf_slam_3d.h
 *  Author: Vladislav Tananaev
 *
 */

#pragma once

#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/system/os.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/slam/CRangeBearingKFSLAM.h>
#include <mrpt/math/ops_containers.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/COpenGLScene.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CSensoryFrame.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CRawlog.h>

/**
 * @brief The EKFslam class provides EKF SLAM 3d from MRPT libraries.
 *
 */
class EKFslam
{
   public:
	/**
	 * @brief constructor
	 */
	EKFslam();
	/**
	 * @brief destructor
	 */
	virtual ~EKFslam();
	/**
	 * @brief init 3D window from mrpt lib
	 */
	void init3Dwindow();
	/**
	 * @brief run 3D window update from mrpt lib
	 */
	void run3Dwindow();
	/**
	 * @brief convert landmark to 3d point
	 */
	void landmark_to_3d(
		const mrpt::slam::CRangeBearingKFSLAM::KFArray_FEAT& lm,
		mrpt::math::TPoint3D& p);
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
	void observation(
		mrpt::obs::CSensoryFrame::Ptr _sf,
		mrpt::obs::CObservationOdometry::Ptr _odometry);

   protected:
	mrpt::slam::CRangeBearingKFSLAM mapping;  ///< EKF slam 3d class

	mrpt::system::TTimeStamp
		timeLastUpdate_;  ///< last update of the pose and map

	mrpt::obs::CActionCollection::Ptr action;  ///< actions
	mrpt::obs::CSensoryFrame::Ptr sf;  ///< observations

	mrpt::poses::CPose3D
		odomLastObservation_;  ///< last observation of odometry
	mrpt::obs::CActionRobotMovement3D::TMotionModelOptions
		motion_model_options_;	///< used with odom value motion noise

	mrpt::gui::CDisplayWindow3D::Ptr win3d;	 ///< MRPT window
	bool SHOW_3D_LIVE;
	bool CAMERA_3DSCENE_FOLLOWS_ROBOT;
	std::vector<mrpt::math::TPose3D> meanPath;
	mrpt::poses::CPose3DQuatPDFGaussian robotPose_;	 ///< current robot pose
	std::vector<mrpt::math::TPoint3D> LMs_;	 ///< vector of the landmarks
	/// vector of the landmarks ID
	std::map<unsigned int, mrpt::maps::CLandmark::TLandmarkID> LM_IDs_;
	mrpt::math::CMatrixDouble fullCov_;	 ///< full covariance matrix
	mrpt::math::CVectorDouble fullState_;  ///< full state vector
};
