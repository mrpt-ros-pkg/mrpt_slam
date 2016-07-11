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
        EKFslam::read_rawlog(data,rawlog_filename);
        rawlog_play_=true;
     }

   
      ///Create publishers///        
      //publish grid map
      //pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
     // pub_metadata_= n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
      //robot pose
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
            sensorSub_[i]  = n_.subscribe(lstSources[i],  1, &PFslamWrapper::callbackBeacon, this);
		}
	}

*/


}

bool EKFslamWrapper::rawlogPlay(){
    if ( rawlog_play_==false){
        return false;    
    }else{

   
   
    for (int i=0; i<data.size(); i++){
      if(ros::ok()){
        
        tictac.Tic();
        mapping.processActionObservation(data[i].first,data[i].second);
       //mapBuilder->processActionObservation( data[i].first,data[i].second);
        t_exec = tictac.Tac();
      	printf("Map building executed in %.03fms\n", 1000.0f*t_exec );
       
        ros::Duration(rawlog_play_delay).sleep(); 
    /*
         metric_map_ = mapBuilder->mapPDF.getCurrentMostLikelyMetricMap();
         mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);
        //if I received new grid maps from 2D laser scan sensors
        if (metric_map_->m_gridMaps.size()){         
              
                nav_msgs::OccupancyGrid  _msg;  
   
                //if we have new map for current sensor update it
               mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg );
                    pub_map_.publish(_msg);
                    pub_metadata_.publish(_msg.info);          
           
        }
   
    //if I received new beacon (range only) map 
        if (metric_map_->m_beaconMap){         

         	mrpt::opengl::CSetOfObjectsPtr objs;
            objs = mrpt::opengl::CSetOfObjects::Create();
            metric_map_->m_beaconMap->getAs3DObject( objs );

            geometry_msgs::PoseArray poseArrayBeacons;
		    poseArrayBeacons.header.frame_id = global_frame_id;
		    poseArrayBeacons.header.stamp = ros::Time::now();
	
		   
	        int objs_counter = 0;
		    while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter )) { 
			    objs_counter++;
		    }	
		    poseArrayBeacons.poses.resize(objs_counter);
		    mrpt::opengl::CEllipsoidPtr beacon_particle;

		     for (size_t i = 0; i < objs_counter; i++) {
			    beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);	
			    mrpt_bridge::convert(beacon_particle->getPose(), poseArrayBeacons.poses[i]);
                viz_beacons.push_back(beacon_particle);		
		    }
		    pub_Particles_Beacons_.publish(poseArrayBeacons);  
            vizBeacons();
            viz_beacons.clear();
        }

        //publish pose
        geometry_msgs::PoseArray poseArray;
		poseArray.header.frame_id =  global_frame_id;
		poseArray.header.stamp = ros::Time::now();
		poseArray.poses.resize(curPDF.particlesCount());
		for(size_t i = 0; i < curPDF.particlesCount(); i++) {
			mrpt::poses::CPose3D p = curPDF.getParticlePose(i);
			mrpt_bridge::convert(p, poseArray.poses[i]);
		}

		pub_Particles_.publish(poseArray);
      }
*/

        }
        ros::spinOnce();
    }

 }   
    return true;
}
