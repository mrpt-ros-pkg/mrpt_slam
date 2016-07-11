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




void EKFslam::read_rawlog(std::vector<std::pair<CActionCollection,CSensoryFrame>>& data, std::string rawlog_filename){
    size_t								rawlogEntry = 0;
	CFileGZInputStream					rawlogFile( rawlog_filename );
    CActionCollectionPtr					action;
	CSensoryFramePtr						observations;

    for (;;)
	{
		if (os::kbhit())
		{
			char c = os::getch();
			if (c==27)
				break;
		}

		// Load action/observation pair from the rawlog:
		// --------------------------------------------------
		if (! CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) ){
			break; // file EOF
        }
        data.push_back( std::make_pair(*action, *observations));
     }
}


