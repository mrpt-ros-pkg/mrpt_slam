/*
 * File: mrpt_ekf_slam_3d.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d.h"

EKFslam::EKFslam(){
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







