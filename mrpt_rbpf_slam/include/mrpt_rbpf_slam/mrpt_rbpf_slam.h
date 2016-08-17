/* 
 *  File: mrpt_slam.h
 *  Author: Vladislav Tananaev
 * 
 *
 */ 

#ifndef MPRT_RBPF_SLAM_H
#define MRPT_RBPF_SLAM_H
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

#include <mrpt/gui/CDisplayWindow3D.h>


using namespace mrpt;
using namespace mrpt::slam;
using namespace mrpt::maps;
using namespace mrpt::obs;
using namespace mrpt::opengl;
using namespace mrpt::gui;
using namespace mrpt::math;
using namespace mrpt::utils;
using namespace mrpt::system;
using namespace mrpt::random;
using namespace mrpt::poses;

/**
 * @brief The PFslam class provides Rao-Blackwellized Particle filter SLAM from MRPT libraries. 
 *   
 */
class PFslam{
public:
   /**
   * @brief constructor
   */
    PFslam();
   /**
   * @brief destructor
   */
   virtual ~PFslam();

   void init3Dwindow();

    void run3Dwindow();


  /**
   * @brief read ini file
   *
   * @param ini_filename the name of the ini file to read
   */
    void read_iniFile(std::string ini_filename);

  /**
   * @brief initialize the SLAM 
   */
    void init_slam();

   /**
   * @brief read pairs of actions and observations from rawlog file 
   *
   * @param data vector of pairs of actions and observations
   * @param rawlog_filename the name of rawlog file to read
   */
    void read_rawlog(std::vector<std::pair<CActionCollection,CSensoryFrame>>& data,std::string rawlog_filename);

   /**
   * @brief calculate the actions from odometry model for current observation
   *
   * @param _sf  current observation
   * @param _odometry raw odometry
   */    
    void observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry);
protected:

    CMetricMapBuilderRBPF* mapBuilder;///< map builder  
    CActionCollectionPtr action;///< actions
	CSensoryFramePtr sf;///< observations

    mrpt::poses::CPose2D odomLastObservation_;  ///< last observation of odometry
    bool use_motion_model_default_options_; ///< used default odom_params
	CActionRobotMovement2D::TMotionModelOptions motion_model_default_options_; ///< used if there are is not odom
	CActionRobotMovement2D::TMotionModelOptions motion_model_options_;         ///< used with odom value motion noise

	

    CMetricMapBuilderRBPF::TConstructionOptions		rbpfMappingOptions;///< options for SLAM from ini file
    mrpt::system::TTimeStamp timeLastUpdate_;///< last update of the pose and map

    CMultiMetricMap *metric_map_; ///<receive map after iteration of SLAM to metric map
    CPose3DPDFParticles   curPDF;///<current robot pose

    mrpt::gui::CDisplayWindow3DPtr	win3D;///<MRPT window 
    bool CAMERA_3DSCENE_FOLLOWS_ROBOT;
    bool SHOW_PROGRESS_IN_WINDOW;
    int SHOW_PROGRESS_IN_WINDOW_DELAY_MS;
    int  PROGRESS_WINDOW_WIDTH, PROGRESS_WINDOW_HEIGHT; 

};

#endif /*MRPT_RBPF_SLAM_H*/
