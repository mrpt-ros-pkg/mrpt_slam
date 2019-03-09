/*
 *  File: mrpt_slam.h
 *  Author: Vladislav Tananaev
 *
 *
 */

#ifndef MRPT_RBPF_SLAM_H
#define MRPT_RBPF_SLAM_H

#include <ros/console.h>

#include <mrpt/gui/CDisplayWindow3D.h>
#include <mrpt/random.h>
#include <mrpt/slam/CMetricMapBuilderRBPF.h>
#include <mrpt/utils/CConfigFile.h>
//#include <mrpt/config/CConfigFile.h> \\ \todo substitute
#include <mrpt/utils/CFileGZInputStream.h>
#include <mrpt/utils/CFileGZOutputStream.h>

#include <mrpt/opengl/CEllipsoid.h>
#include <mrpt/opengl/CGridPlaneXY.h>
#include <mrpt/opengl/CSetOfLines.h>
#include <mrpt/opengl/stock_objects.h>
#include <mrpt/poses/CPose3DPDF.h>
#include <mrpt/poses/CPosePDFGaussian.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>

#include <mrpt/gui/CDisplayWindow3D.h>

#include <mrpt/version.h>
#include <mrpt/maps/CMultiMetricMap.h>
#include <mrpt/obs/CActionCollection.h>
#include <mrpt/obs/CActionRobotMovement2D.h>
#include <mrpt/obs/CActionRobotMovement3D.h>
#include <mrpt/obs/CObservationBeaconRanges.h>
#include <mrpt/obs/CObservationBearingRange.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CRawlog.h>
#include <mrpt/obs/CSensoryFrame.h>

/**
 * @brief The PFslam class provides Rao-Blackwellized Particle filter SLAM from
 * MRPT libraries.
 */
class PFslam
{
public:
  PFslam();
  virtual ~PFslam();

  void init3Dwindow();

  void run3Dwindow();

  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
  void readIniFile(std::string ini_filename);

  /**
   * @brief initialize the SLAM
   */
  void initSlam();

  /**
   * @brief read pairs of actions and observations from rawlog file
   *
   * @param data vector of pairs of actions and observations
   * @param rawlog_filename the name of rawlog file to read
   */
  void read_rawlog(std::vector<std::pair<mrpt::obs::CActionCollection, mrpt::obs::CSensoryFrame>>& data,
                   std::string rawlog_filename);

  /**
   * @brief calculate the actions from odometry model for current observation
   *
   * @param _sf  current observation
   * @param _odometry raw odometry
   */
  void observation(mrpt::obs::CSensoryFrame::Ptr _sf, mrpt::obs::CObservationOdometry::Ptr _odometry);

protected:
  mrpt::slam::CMetricMapBuilderRBPF* mapBuilder;  ///< map builder
  mrpt::obs::CActionCollection::Ptr action;       ///< actions
  mrpt::obs::CSensoryFrame::Ptr sf;               ///< observations

  mrpt::poses::CPose2D odomLastObservation_;  ///< last observation of odometry
  bool use_motion_model_default_options_;     ///< used default odom_params
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_default_options_;  ///< used if there are is not
                                                                                         ///< odom
  mrpt::obs::CActionRobotMovement2D::TMotionModelOptions motion_model_options_;  ///< used with odom value motion noise

  mrpt::slam::CMetricMapBuilderRBPF::TConstructionOptions rbpfMappingOptions;  ///< options for SLAM from ini file
  mrpt::system::TTimeStamp timeLastUpdate_;                                    ///< last update of the pose and map

  const mrpt::maps::CMultiMetricMap* metric_map_;  ///< receive map after iteration of SLAM to metric map
  mrpt::poses::CPose3DPDFParticles curPDF;         ///< current robot pose

  mrpt::gui::CDisplayWindow3D::Ptr win3D;  ///< MRPT window
  bool CAMERA_3DSCENE_FOLLOWS_ROBOT;
  bool SHOW_PROGRESS_IN_WINDOW;
  int SHOW_PROGRESS_IN_WINDOW_DELAY_MS;
  int PROGRESS_WINDOW_WIDTH, PROGRESS_WINDOW_HEIGHT;
};

#endif /*MRPT_RBPF_SLAM_H*/
