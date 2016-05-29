
#include "mrpt_slam/mrpt_slam_wrapper.h"

PFslamWrapper::PFslamWrapper(){
}

PFslamWrapper::~PFslamWrapper(){
}


void PFslamWrapper::get_param(){
    ROS_INFO("READ PARAM FROM LAUNCH FILE");

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


void PFslamWrapper::init(){
     //get parameters from ini file
      PFslam::read_iniFile(ini_filename);
   
      //publish map
      pub_map_ = n_.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
      pub_metadata_= n_.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);

    pub_Particles_ = n_.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);

        //read rawlog file
   PFslam::read_rawlog(data,rawlog_filename);

}



void PFslamWrapper::publishMap() {
    CMetricMapBuilderRBPF mapBuilder1( rbpfMappingOptions );
      PFslam::mapBuilder=mapBuilder1;
       COccupancyGridMap2D  map;
       CPose3DPDFParticles   curPDF;

      for (int i=0; i<data.size(); i++){



      nav_msgs::OccupancyGrid  _msg;
      bool success;

    PFslam::run_slam(data[i].first,data[i].second, curPDF, map);
     success= mrpt_bridge::convert(map, _msg );


         pub_map_.publish(_msg);


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
}
