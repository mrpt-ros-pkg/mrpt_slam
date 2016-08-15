/*
 * File: mrpt_ekf_slam_3d.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d.h"

EKFslam::EKFslam(){
        motion_model_options_.modelSelection = CActionRobotMovement3D::mm6DOF;

        motion_model_options_.mm6DOFModel.nParticlesCount=100;
        motion_model_options_.mm6DOFModel.a1 = 0.114;
        motion_model_options_.mm6DOFModel.a2 =  0.1114;
        motion_model_options_.mm6DOFModel.a3 =  0.114;
        motion_model_options_.mm6DOFModel.a4 =  0.114;
        motion_model_options_.mm6DOFModel.a5 = 0.13;
        motion_model_options_.mm6DOFModel.a6 = 0.13;
        motion_model_options_.mm6DOFModel.a7 =  0.03;
        motion_model_options_.mm6DOFModel.a8 = 0.03;
        motion_model_options_.mm6DOFModel.a9 =0.03;
        motion_model_options_.mm6DOFModel.a10 =  0.03;
        motion_model_options_.mm6DOFModel.additional_std_XYZ  = 0.005;
        motion_model_options_.mm6DOFModel.additional_std_angle =  0.0005;

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

    CActionRobotMovement3D odom_move;


	odom_move.timestamp = _sf->getObservationByIndex(0)->timestamp;
    if(_odometry) {
		
        if(odomLastObservation_.empty()) {
            odomLastObservation_ = _odometry->odometry;
        }
		
        mrpt::poses::CPose3D incOdoPose = _odometry->odometry - odomLastObservation_;
        odomLastObservation_ = _odometry->odometry;
        odom_move.computeFromOdometry(incOdoPose,motion_model_options_);
        action->insert(odom_move);
      
    } 

}






