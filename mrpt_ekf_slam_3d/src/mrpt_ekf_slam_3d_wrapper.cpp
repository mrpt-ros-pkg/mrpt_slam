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

         state_viz_pub_=n_.advertise<visualization_msgs::MarkerArray>("/state_viz", 1);//map




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
             mapping.getCurrentState( robotPose_,LMs_,LM_IDs_,fullState_,fullCov_ );
            // ros::spinOnce();
             viz_state(); 
		}
      
     }
    return true;
 }

}

void EKFslamWrapper::viz_state(){

    visualization_msgs::MarkerArray ma;
    visualization_msgs::Marker marker;

    marker.header.frame_id = "/map";

    //robot pose
    CPose3D robotPoseMean3D = CPose3D(robotPose_.mean);
    tf::Quaternion quat;
    quat=tf::createQuaternionFromRPY(robotPoseMean3D.roll(),robotPoseMean3D.pitch(), robotPoseMean3D.yaw());
    marker.id = 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;
    marker.lifetime = ros::Duration(0);
    marker.pose.position.x = robotPoseMean3D.x();
    marker.pose.position.y = robotPoseMean3D.y();
    marker.pose.position.z = robotPoseMean3D.z();
    marker.pose.orientation.x = quat.x();
    marker.pose.orientation.y = quat.y();
    marker.pose.orientation.z = quat.z();
    marker.pose.orientation.w = quat.w();
    marker.scale.x = 0.12;
    marker.scale.y = 0.12;
    marker.scale.z = 0.12;
    marker.color.a = 1.0;
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    ma.markers.push_back(marker);

    //landmarks
    for (int i=0; i<LMs_.size(); i++) {
        marker.id++;
        marker.color.a = 1.0;
        marker.color.r = 0.0;   
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        marker.type = visualization_msgs::Marker::SPHERE;
        marker.pose.position.x = LMs_[i].x;
        marker.pose.position.y = LMs_[i].y;
        marker.pose.position.z = LMs_[i].z;
            

        ma.markers.push_back(marker);
       

 
        marker.id++;
        marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
        marker.text = std::to_string(LM_IDs_[i]);

        marker.pose.position.x = LMs_[i].x;
        marker.pose.position.y = LMs_[i].y;
        marker.pose.position.z = LMs_[i].z+0.1;
        marker.color.r = 1.0;
        marker.color.g = 1.0;
        marker.color.b = 1.0;
        ma.markers.push_back(marker);
       

}


    state_viz_pub_.publish(ma);


}
