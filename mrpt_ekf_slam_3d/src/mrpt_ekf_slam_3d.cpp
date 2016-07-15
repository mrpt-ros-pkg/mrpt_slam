/*
 * File: mrpt_ekf_slam_3d.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d.h"

EKFslam::EKFslam(){
    use_motion_model_default_options_=false;
    motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
    motion_model_default_options_.gausianModel.minStdXY  = 0.10;
    motion_model_default_options_.gausianModel.minStdPHI = 2.0;

        motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
        motion_model_options_.gausianModel.a1 = 0.034;
        motion_model_options_.gausianModel.a2 = 0.057;
        motion_model_options_.gausianModel.a3 = 0.014;
        motion_model_options_.gausianModel.a4 = 0.097;
        motion_model_options_.gausianModel.minStdXY  = 0.005;
        motion_model_options_.gausianModel.minStdPHI = 0.05;
}

EKFslam::~EKFslam(){
}

void EKFslam::read_iniFile(std::string ini_filename){


      CConfigFile		iniFile( ini_filename );
    
    // Load the config options for mapping:
	// ----------------------------------------
	mapping.loadOptions( iniFile );
	mapping.KF_options.dumpToConsole();
	mapping.options.dumpToConsole();
}

void EKFslam::observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry) {
	action = CActionCollection::Create();
	CActionRobotMovement2D odom_move;
    odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;
	
    if(_odometry) {
		
        if(odomLastObservation_.empty()) {
            odomLastObservation_ = _odometry->odometry;
        }
		
        mrpt::poses::CPose2D incOdoPose = _odometry->odometry - odomLastObservation_;
        odomLastObservation_ = _odometry->odometry;
        odom_move.computeFromOdometry(incOdoPose,motion_model_options_);
        action->insert(odom_move);
      
    } else if(use_motion_model_default_options_){
    
		odom_move.computeFromOdometry(mrpt::poses::CPose2D(0,0,0), motion_model_default_options_);
        action->insert(odom_move);

    }
}






