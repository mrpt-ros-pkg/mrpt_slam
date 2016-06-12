/* 
 *  File: mrpt_slam.cpp
 *  Author: Vladislav Tananaev
 * 
 *
 */ 
#include <mrpt_rbpf_slam/mrpt_rbpf_slam.h>


PFslam::PFslam(){
    use_motion_model_default_options_=false;
    motion_model_default_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
    motion_model_default_options_.gausianModel.minStdXY  = 0.10;
    motion_model_default_options_.gausianModel.minStdPHI = 2.0;

        motion_model_options_.modelSelection = CActionRobotMovement2D::mmGaussian;
        motion_model_options_.gausianModel.a1 =  0.034;
        motion_model_options_.gausianModel.a2 = 0.057;
        motion_model_options_.gausianModel.a3 = 0.014;
        motion_model_options_.gausianModel.a4 = 0.097;
        motion_model_options_.gausianModel.minStdXY  = 0.005;
        motion_model_options_.gausianModel.minStdPHI = 0.05;
    
      

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


void PFslam::observation(CSensoryFramePtr _sf, CObservationOdometryPtr _odometry) {
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
      
   } else {
      if(use_motion_model_default_options_){
        //log_info("No odometry at update %4i -> using dummy", update_counter_);
		odom_move.computeFromOdometry(mrpt::poses::CPose2D(0,0,0), motion_model_default_options_);
        action->insert(odom_move);
         timeLastUpdate_ = _sf->getObservationByIndex(0)->timestamp;
       // updateFilter(action, _sf);
      } else {
        //log_info("No odometry at update %4i -> skipping observation", update_counter_);
      }
    }
}

void PFslam::init_slam(){


	mapBuilder->options.verbose					= true;
	mapBuilder->options.enableMapUpdating		= true;
    mapBuilder->options.debugForceInsertion		= false;

	randomGenerator.randomize();
}    

void PFslam::run_slam(CActionCollection action,CSensoryFrame observations){


	CTicTac								tictac;
	float								t_exec;

			// Execute:
			// ----------------------------------------
			tictac.Tic();
				mapBuilder->processActionObservation( action, observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );


/*
			CPose3DPDFPtr curPDFptr = mapBuilder->getCurrentPoseEstimation();
			
				CPose3DPDFParticlesPtr pp= CPose3DPDFParticlesPtr(curPDFptr);
				curPDF = *pp;	
    
    map=mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
*/
}
