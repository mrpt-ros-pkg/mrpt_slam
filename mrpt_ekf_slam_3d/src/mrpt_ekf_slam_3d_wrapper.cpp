/*
 * File: mrpt_ekf_slam_3d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d_wrapper.h"


EKFslamWrapper::EKFslamWrapper(){
     rawlog_play_=false;
}
EKFslamWrapper::~EKFslamWrapper(){
}
bool EKFslamWrapper::is_file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

void EKFslamWrapper::get_param(){

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
void EKFslamWrapper::init(){
    //get parameters from ini file
     if(!is_file_exists(ini_filename)){
        ROS_ERROR_STREAM("CAN'T READ INI FILE");
        return;
     }

      EKFslam::read_iniFile(ini_filename);
      //read rawlog file if it  exists
    if(is_file_exists(rawlog_filename)){
        ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: "<<rawlog_filename.c_str());
        rawlog_play_=true;
     }

         pub_Particles_Beacons_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud_beacons", 1, true);


}

bool EKFslamWrapper::rawlogPlay(){
    if ( rawlog_play_==false){
        return false;    
    }else{

        size_t								rawlogEntry = 0;
	    CFileGZInputStream					rawlogFile( rawlog_filename );
        CActionCollectionPtr					action;
	    CSensoryFramePtr						observations;

    for (;;)
	{
		if (ros::ok())
		{
			if (!CRawlog::readActionObservationPair( rawlogFile, action, observations, rawlogEntry) ){
			break; // file EOF
             }
             tictac.Tic();
             mapping.processActionObservation( action,observations);
             t_exec = tictac.Tac();
             printf("Map building executed in %.03fms\n", 1000.0f*t_exec );
             ros::Duration(rawlog_play_delay).sleep();
            // mapping.getCurrentState( robotPose_,LMs_,LM_IDs_,fullState_,fullCov_ );
             ros::spinOnce();


            opengl::CSetOfObjectsPtr  objs = opengl::CSetOfObjects::Create();
		    mapping.getAs3DObject(objs);



              geometry_msgs::PoseArray poseArrayBeacons;
		    poseArrayBeacons.header.frame_id = global_frame_id;
		    poseArrayBeacons.header.stamp = ros::Time::now();
	
		   //Count the number of beacons
	        int objs_counter = 0;
		    while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter )) { 
			    objs_counter++;
		    }	
		    poseArrayBeacons.poses.resize(objs_counter);
		    mrpt::opengl::CEllipsoidPtr beacon_particle;
      
		    for (size_t i = 0; i < objs_counter; i++) {
			    beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);	
			    mrpt_bridge::convert(beacon_particle->getPose(), poseArrayBeacons.poses[i]);
		    }
		    pub_Particles_Beacons_.publish(poseArrayBeacons);  
		}
      
     }
 
 
    return true;
 }

}
