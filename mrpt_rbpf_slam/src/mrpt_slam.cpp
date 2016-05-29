/* 
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 * 
 *
 */ 
#include <mrpt_slam/mrpt_slam.h>


PFslam::PFslam(){
}

PFslam::~PFslam(){
}

void PFslam::read_iniFile(std::string ini_filename){

    ini_filename_=ini_filename;
    std::cout<<"READ INI FILE "<<ini_filename_.c_str()<<"\n"; 

    CConfigFile		iniFile( ini_filename_ );
        // -----------------------------------------
		//			Load config from file:
		// -----------------------------------------
    RAWLOG_FILE=iniFile.read_string("MappingApplication","rawlog_file","",  /*Force existence:*/ true);
		METRIC_MAP_CONTINUATION_GRIDMAP_FILE = iniFile.read_string("MappingApplication","METRIC_MAP_CONTINUATION_GRIDMAP_FILE","");

		METRIC_MAP_CONTINUATION_START_POSE.x = iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_X",.0);
		METRIC_MAP_CONTINUATION_START_POSE.y = iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_Y",.0);
		METRIC_MAP_CONTINUATION_START_POSE.phi = DEG2RAD( iniFile.read_double("MappingApplication","METRIC_MAP_CONTINUATION_START_POSE_PHI_DEG",.0) );

    rbpfMappingOptions.loadFromConfigFile(iniFile,"MappingApplication");
	rbpfMappingOptions.dumpToConsole();

}

void PFslam::read_rawlog(std::vector<std::pair<CActionCollection,CSensoryFrame>>& data, std::string rawlog_filename){
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



void PFslam::run_slam(CActionCollection action,CSensoryFrame observations, CPose3DPDFParticles&	curPDF,COccupancyGridMap2D& map){


	CTicTac								tictac;
	float								t_exec;

	mapBuilder.options.verbose					= true;
	mapBuilder.options.enableMapUpdating		= true;
    mapBuilder.options.debugForceInsertion		= false;

	randomGenerator.randomize();

	
	// ----------------------------------------------------------
	//						Map Building
	// ----------------------------------------------------------
	CPose3D  odoPose(0,0,0);


			// Execute:
			// ----------------------------------------
			tictac.Tic();
				mapBuilder.processActionObservation( action, observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );



			CPose3DPDFPtr curPDFptr = mapBuilder.getCurrentPoseEstimation();
			
				CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(curPDFptr);
				curPDF = *pp;	


     map = *mapBuilder.getCurrentlyBuiltMetricMap()->m_gridMaps[0]; 

}
