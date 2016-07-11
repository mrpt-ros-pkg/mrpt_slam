/*
 * File: mrpt_icp_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 * 
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.h"
ICPslamWrapper::ICPslamWrapper(){
      rawlog_play_=false;
}
ICPslamWrapper::~ICPslamWrapper(){

}

bool ICPslamWrapper::is_file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

void ICPslamWrapper::get_param(){
    ROS_INFO("READ PARAM FROM LAUNCH FILE");
    n_.param<double>("rawlog_play_delay", rawlog_play_delay, 0.1);
    ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay);

    n_.getParam("rawlog_filename", rawlog_filename);
    ROS_INFO("rawlog_filename: %s", rawlog_filename.c_str());

    n_.getParam("ini_filename", ini_filename);
    ROS_INFO("ini_filename: %s", ini_filename.c_str());

    n_.param<std::string>("global_frame_id", global_frame_id, "map");
    ROS_INFO("global_frame_id: %s", global_frame_id.c_str());

    n_.param<std::string>("odom_frame_id", odom_frame_id, "odom");
    ROS_INFO("odom_frame_id: %s", odom_frame_id.c_str());

    n_.param<std::string>("base_frame_id", base_frame_id, "base_link");
    ROS_INFO("base_frame_id: %s", base_frame_id.c_str());

    n_.param<std::string>("sensor_source", sensor_source, "scan");
    ROS_INFO("sensor_source: %s", sensor_source.c_str());
    
}

void ICPslamWrapper::init(){

     //get parameters from ini file
     if(!is_file_exists(ini_filename)){
        ROS_ERROR_STREAM("CAN'T READ INI FILE");
        return;
     }
     read_iniFile(ini_filename);
      //read rawlog file if it  exists
    if(is_file_exists(rawlog_filename)){
        ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: "<<rawlog_filename.c_str());
        rawlog_play_=true;
     }

    
      ///Create publishers///        
      //publish grid map
      pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      pub_metadata_= n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      //robot pose
     pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
     // pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
/*  
            
    //read sensor topics
    std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source," ,\t\n",lstSources);
	ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
	

    ///Create subscribers///	
	sensorSub_.resize(lstSources.size());
	for (size_t i=0;i<lstSources.size();i++) {
		if(lstSources[i].find("scan") != std::string::npos) {
			sensorSub_[i]  = n_.subscribe(lstSources[i],  1, &PFslamWrapper::laserCallback, this);
		}
		else {
           std::cout<<"Sensor topics should be 2d laser scans which inlude in the name the word scan "<<"\n";
		}
	}

    //init slam
    mapBuilder = new mrpt::slam::CMetricMapBuilderRBPF(rbpfMappingOptions);
    init_slam();
*/
}
bool ICPslamWrapper::rawlogPlay() {
    if ( rawlog_play_==false){
        return false;    
    }else{
         size_t								rawlogEntry = 0;
	    CFileGZInputStream					rawlogFile( rawlog_filename );
        CActionCollectionPtr	action;
		CSensoryFramePtr		observations;
		CObservationPtr			observation;

    for (;;)
	{
		if (ros::ok())
		{
			if (! CRawlog::getActionObservationPairOrObservation( rawlogFile, action, observations, observation, rawlogEntry) ){
			break; // file EOF
               }
		     const bool isObsBasedRawlog = observation.present();
            // Execute:
			// ----------------------------------------
			tictac.Tic();
			if (isObsBasedRawlog)
					mapBuilder.processObservation( observation );
			else	mapBuilder.processActionObservation( *action, *observations );
			t_exec = tictac.Tac();
			printf("Map building executed in %.03fms\n", 1000.0f*t_exec );
            
                ros::Duration(rawlog_play_delay).sleep(); 
    
            metric_map_ =  mapBuilder.getCurrentlyBuiltMetricMap();

            CPose3D robotPose;
		    mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);
            curPDF=mapBuilder.getCurrentPoseEstimation();
            //publish map
            if (metric_map_->m_gridMaps.size()){         
              
                nav_msgs::OccupancyGrid  _msg;  
   
                //if we have new map for current sensor update it
               mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg );
                    pub_map_.publish(_msg);
                    pub_metadata_.publish(_msg.info);          
           

         }
   

 

    //publish pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = "/map";
   
    //the pose
    pose.pose.position.x = robotPose.x();
    pose.pose.position.y = robotPose.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());

    pub_pose_.publish(pose);
   

        }
   }
        

    return true;
}
}
