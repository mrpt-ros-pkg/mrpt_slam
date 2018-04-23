/*
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 *
 *
 */
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>
#include <mrpt/version.h>
#if MRPT_VERSION >= 0x150
#include <mrpt_bridge/utils.h>
#endif

PFslam::PFslam()
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

  PROGRESS_WINDOW_WIDTH = 600;
  PROGRESS_WINDOW_HEIGHT = 500;
  SHOW_PROGRESS_IN_WINDOW = false;
  SHOW_PROGRESS_IN_WINDOW_DELAY_MS = 0;
  CAMERA_3DSCENE_FOLLOWS_ROBOT = false;
}

PFslam::~PFslam()
{
	try {
		std::string sOutMap = "mrpt_rbpfslam_";
		mrpt::system::TTimeParts parts;
		mrpt::system::timestampToParts(now(), parts, true);
		sOutMap += format("%04u-%02u-%02u_%02uh%02um%02us",
			(unsigned int)parts.year,
			(unsigned int)parts.month,
			(unsigned int)parts.day,
			(unsigned int)parts.hour,
			(unsigned int)parts.minute,
			(unsigned int)parts.second );
		sOutMap += ".simplemap";

		sOutMap = mrpt::system::fileNameStripInvalidChars( sOutMap );
		ROS_INFO("Saving built map to `%s`", sOutMap.c_str());
		mapBuilder->saveCurrentMapToFile(sOutMap);
	} catch (std::exception &e) {
		ROS_ERROR("Exception: %s",e.what());
	}
}

void PFslam::read_iniFile(std::string ini_filename)
{
  CConfigFile iniFile(ini_filename);
  rbpfMappingOptions.loadFromConfigFile(iniFile, "MappingApplication");
  rbpfMappingOptions.dumpToConsole();

  // Display variables
  CAMERA_3DSCENE_FOLLOWS_ROBOT = iniFile.read_bool("MappingApplication", "CAMERA_3DSCENE_FOLLOWS_ROBOT", true);
  SHOW_PROGRESS_IN_WINDOW = iniFile.read_bool("MappingApplication", "SHOW_PROGRESS_IN_WINDOW", false);
  SHOW_PROGRESS_IN_WINDOW_DELAY_MS = iniFile.read_int("MappingApplication", "SHOW_PROGRESS_IN_WINDOW_DELAY_MS", 1);

  MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_WIDTH, int, iniFile, "MappingApplication");
  MRPT_LOAD_CONFIG_VAR(PROGRESS_WINDOW_HEIGHT, int, iniFile, "MappingApplication");
}

void PFslam::read_rawlog(std::vector<std::pair<CActionCollection, CSensoryFrame>> &data, std::string rawlog_filename)
{
  size_t rawlogEntry = 0;
#if MRPT_VERSION>=0x199
  #include <mrpt/serialization/CArchive.h>
  CFileGZInputStream rawlog_stream(rawlog_filename);
  auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);
#else
  CFileGZInputStream rawlogFile(rawlog_filename);
#endif
  CActionCollection::Ptr action;
  CSensoryFrame::Ptr observations;

  for (;;)
  {
    if (os::kbhit())
    {
      char c = os::getch();
      if (c == 27)
        break;
    }

    // Load action/observation pair from the rawlog:
    // --------------------------------------------------
    if (!CRawlog::readActionObservationPair(rawlogFile, action, observations, rawlogEntry))
    {
      break;  // file EOF
    }
    data.push_back(std::make_pair(*action, *observations));
  }
}

void PFslam::observation(CSensoryFrame::Ptr _sf, CObservationOdometry::Ptr _odometry)
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

void PFslam::init_slam()
{
#if MRPT_VERSION < 0x150
  mapBuilder->options.verbose = true;
#else
  log4cxx::LoggerPtr ros_logger = log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapBuilder->setVerbosityLevel(mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  mapBuilder->logging_enable_console_output = false;
#if MRPT_VERSION < 0x199
  mapBuilder->logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback_mrpt_15));
#else
  mapBuilder->logRegisterCallback(static_cast<output_logger_callback_t>(&mrpt_bridge::mrptToROSLoggerCallback));
#endif
#endif

  mapBuilder->options.enableMapUpdating = true;
  mapBuilder->options.debugForceInsertion = false;

#if MRPT_VERSION >= 0x199
  getRandomGenerator().randomize();
#else
randomGenerator.randomize();
#endif
}

void PFslam::init3Dwindow()
{
#if MRPT_HAS_WXWIDGETS

  if (SHOW_PROGRESS_IN_WINDOW)
  {
    win3D = mrpt::gui::CDisplayWindow3D::Create("RBPF-SLAM @ MRPT C++ Library", PROGRESS_WINDOW_WIDTH,
                                                PROGRESS_WINDOW_HEIGHT);
    win3D->setCameraZoom(40);
    win3D->setCameraAzimuthDeg(-50);
    win3D->setCameraElevationDeg(70);
  }

#endif
}
void PFslam::run3Dwindow()
{
  // Save a 3D scene view of the mapping process:
  if (SHOW_PROGRESS_IN_WINDOW && win3D)
  {
    // get the current map and pose
    metric_map_ = mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
    mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);
    COpenGLScene::Ptr scene;
    scene = COpenGLScene::Create();

    // The ground:
    mrpt::opengl::CGridPlaneXY::Ptr groundPlane = mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
    groundPlane->setColor(0.4, 0.4, 0.4);
    scene->insert(groundPlane);

    // The camera pointing to the current robot pose:
    if (CAMERA_3DSCENE_FOLLOWS_ROBOT)
    {
      mrpt::opengl::CCamera::Ptr objCam = mrpt::opengl::CCamera::Create();
      CPose3D robotPose;
      curPDF.getMean(robotPose);

      objCam->setPointingAt(robotPose);
      objCam->setAzimuthDegrees(-30);
      objCam->setElevationDegrees(30);
      scene->insert(objCam);
    }
    // Draw the map(s):
    mrpt::opengl::CSetOfObjects::Ptr objs = mrpt::opengl::CSetOfObjects::Create();
    metric_map_->getAs3DObject(objs);
    scene->insert(objs);

    // Draw the robot particles:
    size_t M = mapBuilder->mapPDF.particlesCount();
    mrpt::opengl::CSetOfLines::Ptr objLines = mrpt::opengl::CSetOfLines::Create();
    objLines->setColor(0, 1, 1);
    for (size_t i = 0; i < M; i++)
    {
      std::deque<TPose3D> path;
      mapBuilder->mapPDF.getPath(i, path);

      float x0 = 0, y0 = 0, z0 = 0;
      for (size_t k = 0; k < path.size(); k++)
      {
        objLines->appendLine(x0, y0, z0 + 0.001, path[k].x, path[k].y, path[k].z + 0.001);
        x0 = path[k].x;
        y0 = path[k].y;
        z0 = path[k].z;
      }
    }
    scene->insert(objLines);

    // An ellipsoid:
    CPose3D lastMeanPose;
    float minDistBtwPoses = -1;
    std::deque<TPose3D> dummyPath;
    mapBuilder->mapPDF.getPath(0, dummyPath);
    for (int k = (int)dummyPath.size() - 1; k >= 0; k--)
    {
      CPose3DPDFParticles poseParts;
      mapBuilder->mapPDF.getEstimatedPosePDFAtTime(k, poseParts);
      CPose3D meanPose;
      CMatrixDouble66 COV;
      poseParts.getCovarianceAndMean(COV, meanPose);

      if (meanPose.distanceTo(lastMeanPose) > minDistBtwPoses)
      {
        CMatrixDouble33 COV3 = COV.block(0, 0, 3, 3);

        minDistBtwPoses = 6 * sqrt(COV3(0, 0) + COV3(1, 1));

        opengl::CEllipsoid::Ptr objEllip = opengl::CEllipsoid::Create();
        objEllip->setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001);
        objEllip->setCovMatrix(COV3, COV3(2, 2) == 0 ? 2 : 3);

        objEllip->setColor(0, 0, 1);
        objEllip->enableDrawSolid3D(false);
        scene->insert(objEllip);

        lastMeanPose = meanPose;
      }
    }

    COpenGLScene::Ptr &scenePtr = win3D->get3DSceneAndLock();
    scenePtr = scene;
    win3D->unlockAccess3DScene();
    win3D->forceRepaint();
  }
}
