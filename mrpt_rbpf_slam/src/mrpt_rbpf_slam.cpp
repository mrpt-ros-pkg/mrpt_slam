/*
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 *
 *
 */
#include <mrpt/version.h>
#include <mrpt_bridge/utils.h>
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>

#if MRPT_VERSION >= 0x199
#include <mrpt/serialization/CArchive.h>
#endif

namespace mrpt_rbpf_slam {
PFslam::~PFslam() {
  try {
    std::string sOutMap = "mrpt_rbpfslam_";
    mrpt::system::TTimeParts parts;
    mrpt::system::timestampToParts(mrpt::system::now(), parts, true);
    sOutMap +=
        mrpt::format("%04u-%02u-%02u_%02uh%02um%02us", (unsigned int)parts.year,
                     (unsigned int)parts.month, (unsigned int)parts.day,
                     (unsigned int)parts.hour, (unsigned int)parts.minute,
                     (unsigned int)parts.second);
    sOutMap += ".simplemap";

    sOutMap = mrpt::system::fileNameStripInvalidChars(sOutMap);
    if (mrpt::system::directoryExists(options_.simplemap_path_prefix)) {
      sOutMap = options_.simplemap_path_prefix + sOutMap;
    } else {
      std::cerr << "Folder " << options_.simplemap_path_prefix
                << " doesn't exist, cannot save simplemap there! Default "
                   "'~/.ros/' path will be used.\n";
    }
    std::cout << "Saving built map " << sOutMap << "\n";
    mapBuilder_.saveCurrentMapToFile(sOutMap);
  } catch (const std::exception &e) {
    std::cerr << "Exception: " << e.what() << "\n";
  }
}

void PFslam::readIniFile(const std::string &ini_filename) {
#if MRPT_VERSION >= 0x199
  mrpt::config::CConfigFile iniFile(ini_filename);
#else
  mrpt::utils::CConfigFile iniFile(ini_filename);
#endif
  options_.rbpfMappingOptions_.loadFromConfigFile(iniFile,
                                                  "MappingApplication");
  options_.rbpfMappingOptions_.dumpToConsole();
}

void PFslam::readRawlog(
    const std::string &rawlog_filename,
    std::vector<std::pair<mrpt::obs::CActionCollection,
                          mrpt::obs::CSensoryFrame>> &data) {
  size_t rawlogEntry = 0;
#if MRPT_VERSION >= 0x199
  mrpt::io::CFileGZInputStream rawlog_stream(rawlog_filename);
  auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);
#else
  mrpt::utils::CFileGZInputStream rawlogFile(rawlog_filename);
#endif
  mrpt::obs::CActionCollection::Ptr action;
  mrpt::obs::CSensoryFrame::Ptr observations;

  for (;;) {
    if (mrpt::system::os::kbhit()) {
      char c = mrpt::system::os::getch();
      if (c == 27)
        break;
    }

    // Load action/observation pair from the rawlog:
    // --------------------------------------------------
    if (!mrpt::obs::CRawlog::readActionObservationPair(
            rawlogFile, action, observations, rawlogEntry)) {
      break; // file EOF
    }
    data.emplace_back(*action, *observations);
  }
}

void PFslam::observation(
    const mrpt::obs::CSensoryFrame::ConstPtr sensory_frame,
    const mrpt::obs::CObservationOdometry::ConstPtr odometry) {
  action_ = mrpt::obs::CActionCollection::Create();
  mrpt::obs::CActionRobotMovement2D odom_move;
  odom_move.timestamp = sensory_frame->getObservationByIndex(0)->timestamp;

  if (odometry) {
    if (odomLastObservation_.empty()) {
      odomLastObservation_ = odometry->odometry;
    }

    mrpt::poses::CPose2D incOdoPose = odometry->odometry - odomLastObservation_;
    odomLastObservation_ = odometry->odometry;
    odom_move.computeFromOdometry(incOdoPose, options_.motion_model_options_);
    action_->insert(odom_move);
  }
}

void PFslam::initSlam(PFslam::Options options) {
  log4cxx::LoggerPtr ros_logger =
      log4cxx::Logger::getLogger(ROSCONSOLE_DEFAULT_NAME);
  mapBuilder_.setVerbosityLevel(
      mrpt_bridge::rosLoggerLvlToMRPTLoggerLvl(ros_logger->getLevel()));
  mapBuilder_.logging_enable_console_output = false;
#if MRPT_VERSION < 0x199
  mapBuilder_.logRegisterCallback(static_cast<output_logger_callback_t>(
      &mrpt_bridge::mrptToROSLoggerCallback_mrpt_15));
#else
  mapBuilder_.logRegisterCallback(static_cast<output_logger_callback_t>(
      &mrpt_bridge::mrptToROSLoggerCallback));
#endif
  mapBuilder_.options.enableMapUpdating = true;
  mapBuilder_.options.debugForceInsertion = false;

#if MRPT_VERSION >= 0x199
  mrpt::random::getRandomGenerator().randomize();
#else
  mrpt::random::randomGenerator.randomize();
#endif

  options_ = std::move(options);

  //  use_motion_model_default_options_ = false;
  //  //  motion_model_default_options_.modelSelection =
  //  mrpt::obs::CActionRobotMovement2D::mmGaussian;
  //  //  motion_model_default_options_.gaussianModel.minStdXY = 0.10;
  //  //  motion_model_default_options_.gaussianModel.minStdPHI = 2.0;

  //  motion_model_options_.modelSelection =
  //  mrpt::obs::CActionRobotMovement2D::mmGaussian;
  //  motion_model_options_.gaussianModel.a1 = 0.034f;
  //  motion_model_options_.gaussianModel.a2 = 0.057f;
  //  motion_model_options_.gaussianModel.a3 = 0.014f;
  //  motion_model_options_.gaussianModel.a4 = 0.097f;
  //  motion_model_options_.gaussianModel.minStdXY = 0.005f;
  //  motion_model_options_.gaussianModel.minStdPHI = 0.05f;

  //  PROGRESS_WINDOW_WIDTH_ = 600;
  //  PROGRESS_WINDOW_HEIGHT_ = 500;
  //  SHOW_PROGRESS_IN_WINDOW_ = false;
  //  SHOW_PROGRESS_IN_WINDOW_DELAY_MS_ = 0;
  //  CAMERA_3DSCENE_FOLLOWS_ROBOT_ = false;
}

void PFslam::init3Dwindow() {
#if MRPT_HAS_WXWIDGETS
  if (options_.SHOW_PROGRESS_IN_WINDOW_) {
    win3D_ = mrpt::gui::CDisplayWindow3D::Create(
        "RBPF-SLAM @ MRPT C++ Library", options_.PROGRESS_WINDOW_WIDTH_,
        options_.PROGRESS_WINDOW_HEIGHT_);
    win3D_->setCameraZoom(40);
    win3D_->setCameraAzimuthDeg(-50);
    win3D_->setCameraElevationDeg(70);
  }
#endif
}
void PFslam::run3Dwindow() {
  // Save a 3D scene view of the mapping process:
  if (options_.SHOW_PROGRESS_IN_WINDOW_ && win3D_) {
    // get the current map and pose
    metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
    mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);
    mrpt::opengl::COpenGLScene::Ptr scene;
    scene = mrpt::opengl::COpenGLScene::Create();

    // The ground:
    mrpt::opengl::CGridPlaneXY::Ptr groundPlane =
        mrpt::opengl::CGridPlaneXY::Create(-200, 200, -200, 200, 0, 5);
    groundPlane->setColor(0.4, 0.4, 0.4);
    scene->insert(groundPlane);

    // The camera pointing to the current robot pose:
    if (options_.CAMERA_3DSCENE_FOLLOWS_ROBOT_) {
      mrpt::opengl::CCamera::Ptr objCam = mrpt::opengl::CCamera::Create();
      mrpt::poses::CPose3D robotPose;
      curPDF.getMean(robotPose);

      objCam->setPointingAt(robotPose);
      objCam->setAzimuthDegrees(-30);
      objCam->setElevationDegrees(30);
      scene->insert(objCam);
    }
    // Draw the map(s):
    mrpt::opengl::CSetOfObjects::Ptr objs =
        mrpt::opengl::CSetOfObjects::Create();
    metric_map_->getAs3DObject(objs);
    scene->insert(objs);

    // Draw the robot particles:
    size_t M = mapBuilder_.mapPDF.particlesCount();
    mrpt::opengl::CSetOfLines::Ptr objLines =
        mrpt::opengl::CSetOfLines::Create();
    objLines->setColor(0, 1, 1);
    for (size_t i = 0; i < M; i++) {
      std::deque<mrpt::math::TPose3D> path;
      mapBuilder_.mapPDF.getPath(i, path);

      float x0 = 0, y0 = 0, z0 = 0;
      for (size_t k = 0; k < path.size(); k++) {
        objLines->appendLine(x0, y0, z0 + 0.001, path[k].x, path[k].y,
                             path[k].z + 0.001);
        x0 = path[k].x;
        y0 = path[k].y;
        z0 = path[k].z;
      }
    }
    scene->insert(objLines);

    // An ellipsoid:
    mrpt::poses::CPose3D lastMeanPose;
    float minDistBtwPoses = -1;
    std::deque<mrpt::math::TPose3D> dummyPath;
    mapBuilder_.mapPDF.getPath(0, dummyPath);
    for (int k = (int)dummyPath.size() - 1; k >= 0; k--) {
      mrpt::poses::CPose3DPDFParticles poseParts;
      mapBuilder_.mapPDF.getEstimatedPosePDFAtTime(k, poseParts);
      mrpt::poses::CPose3D meanPose;
      mrpt::math::CMatrixDouble66 COV;
      poseParts.getCovarianceAndMean(COV, meanPose);

      if (meanPose.distanceTo(lastMeanPose) > minDistBtwPoses) {
        mrpt::math::CMatrixDouble33 COV3 = COV.block(0, 0, 3, 3);

        minDistBtwPoses = 6 * sqrt(COV3(0, 0) + COV3(1, 1));

        mrpt::opengl::CEllipsoid::Ptr objEllip =
            mrpt::opengl::CEllipsoid::Create();
        objEllip->setLocation(meanPose.x(), meanPose.y(), meanPose.z() + 0.001);
        objEllip->setCovMatrix(COV3, COV3(2, 2) == 0 ? 2 : 3);

        objEllip->setColor(0, 0, 1);
        objEllip->enableDrawSolid3D(false);
        scene->insert(objEllip);

        lastMeanPose = meanPose;
      }
    }

    mrpt::opengl::COpenGLScene::Ptr &scenePtr = win3D_->get3DSceneAndLock();
    scenePtr = scene;
    win3D_->unlockAccess3DScene();
    win3D_->forceRepaint();
  }
}
} // namespace mrpt_rbpf_slam
