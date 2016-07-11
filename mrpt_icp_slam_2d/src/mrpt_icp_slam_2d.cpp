/*
 * File: mrpt_icp_slam_2d.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d.h"

ICPslam::ICPslam(){
}

ICPslam::~ICPslam(){
}


void ICPslam::read_iniFile(std::string ini_filename){

    	CConfigFile				iniFile(ini_filename);

    mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
	mapBuilder.initialize();

    mapBuilder.options.verbose = true;
    mapBuilder.options.alwaysInsertByClass.fromString( iniFile.read_string("MappingApplication","alwaysInsertByClass","") );


	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();


}

