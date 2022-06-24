/*
 *  File: mrpt_rbpf_slam.h
 *  Author: Vladislav Tananaev
 */

#pragma once
#include <ros/console.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/config/CConfigFile.h>
#include <mrpt/io/CFileGZInputStream.h>
#include <mrpt/io/CFileGZOutputStream.h>
#include <mrpt/opengl/CEllipsoid3D.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>

namespace mrpt_rbpf_slam
{
/**
 * @brief The PFslam class provides Rao-Blackwellized Particle filter SLAM from
 * MRPT libraries.
 */
class PFslam
{
   public:
	struct Options
	{
		mrpt::obs::CActionRobotMovement2D::TMotionModelOptions
			motion_model_options_;	///< used with odom value motion
									///< noise
		mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions
			rbpfMappingOptions_;  ///< options for SLAM from ini file
		bool CAMERA_3DSCENE_FOLLOWS_ROBOT_;
		bool SHOW_PROGRESS_IN_WINDOW_;
		int SHOW_PROGRESS_IN_WINDOW_DELAY_MS_;
		int PROGRESS_WINDOW_WIDTH_, PROGRESS_WINDOW_HEIGHT_;
		std::string simplemap_path_prefix;
	} options_;

	PFslam() = default;
	virtual ~PFslam();

	void init3Dwindow();

	void run3Dwindow();

	/**
	 * @brief Read ini file
	 *
	 * @param[in] ini_filename the name of the ini file to read
	 */
	void readIniFile(const std::string& ini_filename);

	/**
	 * @brief initialize the SLAM
	 */
	void initSlam(Options options);

	/**
	 * @brief Read pairs of actions and observations from rawlog file
	 *
	 * @param[in] rawlog_filename the name of rawlog file to read
	 * @param[out] data vector of pairs of actions and observations
	 */
	void readRawlog(
		const std::string& rawlog_filename,
		std::vector<
			std::pair<mrpt::obs::CActionCollection, mrpt::obs::CSensoryFrame>>&
			data);

	/**
	 * @brief Calculate the actions from odometry model for current observation
	 *
	 * @param[in] sensory_frame  current observation
	 * @param[in] odometry raw odometry
	 */
	void observation(
		const mrpt::obs::CSensoryFrame::ConstPtr sensory_frame,
		const mrpt::obs::CObservationOdometry::ConstPtr odometry);

   protected:
	mrpt::slam::CMetricMapBuilderRBPF mapBuilder_;	///< map builder
	mrpt::obs::CActionCollection::Ptr action_;	///< actions
	mrpt::obs::CSensoryFrame::Ptr sensory_frame_;  ///< observations

	mrpt::poses::CPose2D
		odomLastObservation_;  ///< last observation of odometry
	bool use_motion_model_default_options_;	 ///< used default odom_params
	mrpt::system::TTimeStamp
		timeLastUpdate_;  ///< last update of the pose and map

	const mrpt::maps::CMultiMetricMap*
		metric_map_;  ///< receive map after iteration of SLAM to metric map
	mrpt::poses::CPose3DPDFParticles curPDF;  ///< current robot pose

	mrpt::gui::CDisplayWindow3D::Ptr win3D_;  ///< MRPT window
};
}  // namespace mrpt_rbpf_slam
