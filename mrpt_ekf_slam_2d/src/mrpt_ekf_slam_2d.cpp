/*
 * File: mrpt_ekf_slam_2d.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_2d/mrpt_ekf_slam_2d.h"
#if MRPT_VERSION >= 0x150
#include <mrpt_bridge/utils.h>
#endif

EKFslam::EKFslam()
{
#if MRPT_VERSION>=0x150
#define gausianModel gaussianModel    // a typo was fixed in 1.5.0
#endif

  use_motion_model_default_options_ = false;
  motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_default_options_.gausianModel.minStdXY = 0.10;
  motion_model_default_options_.gausianModel.minStdPHI = 2.0;

  motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
  motion_model_options_.gausianModel.a1 = 0.034;
  motion_model_options_.gausianModel.a2 = 0.057;
  motion_model_options_.gausianModel.a3 = 0.014;
  motion_model_options_.gausianModel.a4 = 0.097;
  motion_model_options_.gausianModel.minStdXY = 0.005;
  motion_model_options_.gausianModel.minStdPHI = 0.05;

  // display values
  SHOW_3D_LIVE = false;
  CAMERA_3DSCENE_FOLLOWS_ROBOT = false;
}

EKFslam::~EKFslam()
{
}

void EKFslam::read_iniFile(std::string ini_filename)
{
  CConfigFile iniFile(ini_filename);

  // Load the config options for mapping:
  // ----------------------------------------
  mapping.loadOptions(iniFile);
  mapping.KF_options.dumpToConsole();
  mapping.options.dumpToConsole();

#if MRPT_VERSION >= 0x150
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapping.setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  mapping.logging_enable_console_output = false;

#if MRPT_VERSION >= 0x199
  mapping.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#else
  mapping.logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback_mrpt_15));
#endif
#endif

  // read display variables
  SHOW_3D_LIVE = iniFile.read_bool("MappingApplication", "SHOW_3D_LIVE", false);
  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", false);
}

void EKFslam::observation(CSensoryFrame::Ptr _sf, CObservationOdometry::Ptr _odometry)
{
  action = CActionCollection::Create();
  CActionRobotMovement2D odom_move;
  odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;

  if (_odometry)
  {
    if (odomLastObservation_.empty())
    {
      odomLastObservation_ = _odometry->odometry;
    }

    mrpt::poses::CPose2D incOdoPose = _odometry->odometry - odomLastObservation_;
    odomLastObservation_ = _odometry->odometry;
    odom_move.computeFromOdometry(incOdoPose, motion_model_options_);
    action->insert(odom_move);
  }
  else if (use_motion_model_default_options_)
  {
    odom_move.computeFromOdometry(mrpt::poses::CPose2D(0, 0, 0), motion_model_default_options_);
    action->insert(odom_move);
  }
}

void EKFslam::init3Dwindow()
{
#if MRPT_HAS_WXWIDGETS
  if (SHOW_3D_LIVE)
  {
    win3d = mrpt::gui::CDisplayWindow3D::Create("KF-SLAM live view", 800, 500);
  }
#endif
}

void EKFslam::run3Dwindow()
{
  // Save 3D view of the filter state:
  if (SHOW_3D_LIVE && win3d)
  {
    mapping.getCurrentState(robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);
    // Most of this code was copy and pase from ros::amcl

    // Get the mean robot pose as 3D:
    const CPose3D robotPoseMean3D = CPose3D(robotPose_.mean);

    // Build the path:
	meanPath.push_back(mrpt_bridge::p2t(robotPoseMean3D));

    // create the scene
    COpenGLScene::Ptr scene3D = COpenGLScene::Create();
    opengl::CGridPlaneXY::Ptr grid = opengl::CGridPlaneXY::Create(-1000, 1000, -1000, 1000, 0, 5);
    grid->setColor(0.4, 0.4, 0.4);
    scene3D->insert(grid);

    // Robot path:
    opengl::CSetOfLines::Ptr linesPath = opengl::CSetOfLines::Create();
    linesPath->setColor(1, 0, 0);
    TPose3D init_pose;
    if (!meanPath.empty())
    {
	  init_pose = mrpt_bridge::p2t(CPose3D(meanPath[0]));
      int path_decim = 0;
      for (vector<TPose3D>::iterator it = meanPath.begin(); it != meanPath.end(); ++it)
      {
        linesPath->appendLine(init_pose, *it);
        init_pose = *it;
        if (++path_decim > 10)
        {
          path_decim = 0;
          mrpt::opengl::CSetOfObjects::Ptr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(0.3f, 2.0f);
          xyz->setPose(CPose3D(*it));
          scene3D->insert(xyz);
        }
      }
      scene3D->insert(linesPath);
    }

    // finally a big corner for the latest robot pose:
    mrpt::opengl::CSetOfObjects::Ptr xyz = mrpt::opengl::stock_objects::CornerXYZSimple(1.0, 2.5);
    xyz->setPose(robotPoseMean3D);
    scene3D->insert(xyz);

    // The camera pointing to the current robot pose:
    if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
    {
      win3d->setCameraPointingToPoint(robotPoseMean3D.x(), robotPoseMean3D.y(), robotPoseMean3D.z());
    }

    // Draw latest data association:
    const CRangeBearingKFSLAM2D::TDataAssocInfo &da = mapping.getLastDataAssociation();
    mrpt::opengl::CSetOfLines::Ptr lins = mrpt::opengl::CSetOfLines::Create();
    lins->setLineWidth(1.2);
    lins->setColor(1, 1, 1);
    for (std::map<observation_index_t, prediction_index_t>::const_iterator it = da.results.associations.begin();
         it != da.results.associations.end(); ++it)
    {
      const prediction_index_t idxPred = it->second;
      // This index must match the internal list of features in the map:
      CRangeBearingKFSLAM2D::KFArray_FEAT featMean;
      mapping.getLandmarkMean(idxPred, featMean);

      TPoint3D featMean3D;
      landmark_to_3d(featMean, featMean3D);
      // Line: robot -> landmark:
      lins->appendLine(robotPoseMean3D.x(), robotPoseMean3D.y(), robotPoseMean3D.z(), featMean3D.x, featMean3D.y,
                       featMean3D.z);
    }
    scene3D->insert(lins);

    // The current state of KF-SLAM:
    opengl::CSetOfObjects::Ptr objs = opengl::CSetOfObjects::Create();
    mapping.getAs3DObject(objs);
    scene3D->insert(objs);

    mrpt::opengl::COpenGLScene::Ptr &scn = win3d->get3DSceneAndLock();
    scn = scene3D;

    win3d->unlockAccess3DScene();
    win3d->repaint();
  }
}
void EKFslam::landmark_to_3d(const CRangeBearingKFSLAM2D::KFArray_FEAT &lm, TPoint3D &p)
{
  p.x = lm[0];
  p.y = lm[1];
  p.z = 0;
}
