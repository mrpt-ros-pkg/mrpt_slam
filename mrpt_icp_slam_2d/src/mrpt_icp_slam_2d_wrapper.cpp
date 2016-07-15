/*
 * File: mrpt_icp_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 * 
 */

#include "mrpt_icp_slam_2d/mrpt_icp_slam_2d_wrapper.h"
ICPslamWrapper::ICPslamWrapper(){
      rawlog_play_=false;
      stamp=ros::Time(0);
}
ICPslamWrapper::~ICPslamWrapper(){

}
bool ICPslamWrapper::is_file_exists(const std::string& name) {
    std::ifstream f(name.c_str());
    return f.good();
}

void ICPslamWrapper::read_iniFile(std::string ini_filename){

    	CConfigFile				iniFile(ini_filename);

    mapBuilder.ICP_options.loadFromConfigFile( iniFile, "MappingApplication");
	mapBuilder.ICP_params.loadFromConfigFile ( iniFile, "ICP");
	mapBuilder.initialize();

    mapBuilder.options.verbose = true;
    mapBuilder.options.alwaysInsertByClass.fromString( iniFile.read_string("MappingApplication","alwaysInsertByClass","") );


	mapBuilder.ICP_params.dumpToConsole();
	mapBuilder.ICP_options.dumpToConsole();


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
      //publish point map
      pub_point_cloud_ = n_.advertise<sensor_msgs::PointCloud>("PointCloudMap", 1, true);

      //robot pose
      pub_pose_ = n_.advertise<geometry_msgs::PoseStamped>("robot_pose", 1);
 
 
            
    //read sensor topics
    std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source," ,\t\n",lstSources);
	ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
	

    ///Create subscribers///	
	sensorSub_.resize(lstSources.size());
	for (size_t i=0;i<lstSources.size();i++) {
		if(lstSources[i].find("scan") != std::string::npos) {
			sensorSub_[i]  = n_.subscribe(lstSources[i],  1, &ICPslamWrapper::laserCallback, this);
		}
		else {
           std::cout<<"Sensor topics should be 2d laser scans which inlude in the name the word scan "<<"\n";
		}
	}
}

void ICPslamWrapper::laserCallback(const sensor_msgs::LaserScan &_msg) {
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif
	CObservation2DRangeScanPtr laser = CObservation2DRangeScan::Create();
    if(laser_poses_.find(_msg.header.frame_id) == laser_poses_.end()) { 
        updateSensorPose (_msg.header.frame_id);    
    } else {        
        mrpt::poses::CPose3D pose = laser_poses_[_msg.header.frame_id];
        mrpt_bridge::convert(_msg, laser_poses_[_msg.header.frame_id],  *laser);
	    CObservationPtr obs = CObservationPtr(laser);
        stamp=ros::Time(0);
        tictac.Tic();
        mapBuilder.processObservation( obs );
        t_exec = tictac.Tac();
      	printf("Map building executed in %.03fms\n", 1000.0f*t_exec );
        publishMapPose();

    }
}
void   ICPslamWrapper::publishMapPose(){

 metric_map_ =  mapBuilder.getCurrentlyBuiltMetricMap();             
            //publish map
            if (metric_map_->m_gridMaps.size()){                      
                nav_msgs::OccupancyGrid  _msg;   
                //if we have new map for current sensor update it
               mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg );
                    pub_map_.publish(_msg);
                    pub_metadata_.publish(_msg.info);          
           
         }
           if (metric_map_->m_pointsMaps.size()){         
              
                sensor_msgs::PointCloud  _msg;  
                 std_msgs::Header header;
                 header.stamp=ros::Time(0);
                header.frame_id=global_frame_id;
                //if we have new map for current sensor update it
              mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0],header, _msg );
              pub_point_cloud_ .publish(_msg);
         }

            CPose3D robotPose;
		    mapBuilder.getCurrentPoseEstimation()->getMean(robotPose);

         //publish pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_id;
   
    //the pose
    pose.pose.position.x = robotPose.x();
    pose.pose.position.y = robotPose.y();
    pose.pose.position.z = 0.0;
    pose.pose.orientation = tf::createQuaternionMsgFromYaw(robotPose.yaw());

    pub_pose_.publish(pose);

}
void ICPslamWrapper::updateSensorPose (std::string _frame_id) {
    CPose3D pose;
    tf::StampedTransform transform;
    try {

        listenerTF_.lookupTransform(base_frame_id, _frame_id, ros::Time(0), transform);

        tf::Vector3 translation = transform.getOrigin();
        tf::Quaternion quat = transform.getRotation();
        pose.x() = translation.x();
        pose.y() = translation.y();
        pose.z() = translation.z();
        double roll, pitch, yaw;
        tf::Matrix3x3 Rsrc(quat);
        CMatrixDouble33 Rdes;
        for(int c = 0; c < 3; c++)
            for(int r = 0; r < 3; r++)
                Rdes(r,c) = Rsrc.getRow(r)[c];
        pose.setRotationMatrix(Rdes);
        laser_poses_[_frame_id] = pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

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
            
            //publish map
            if (metric_map_->m_gridMaps.size()){         
              
                nav_msgs::OccupancyGrid  _msg;  
   
                //if we have new map for current sensor update it
               mrpt_bridge::convert(*metric_map_->m_gridMaps[0], _msg );
                    pub_map_.publish(_msg);
                    pub_metadata_.publish(_msg.info);          
           

         }
   
           if (metric_map_->m_pointsMaps.size()){         
              
                sensor_msgs::PointCloud  _msg;  
                std_msgs::Header header;
                 header.stamp=ros::Time(0);
                header.frame_id=global_frame_id;
                //if we have new map for current sensor update it
               mrpt_bridge::point_cloud::mrpt2ros(*metric_map_->m_pointsMaps[0],header, _msg );
                   pub_point_cloud_ .publish(_msg);
                    //pub_metadata_.publish(_msg.info)      
           

         }
 

    //publish pose
    geometry_msgs::PoseStamped pose;
    pose.header.frame_id = global_frame_id;
   
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




void ICPslamWrapper::publishTF() {
	// Most of this code was copy and pase from ros::amcl
	mrpt::poses::CPose3D		robotPoseTF;
	mapBuilder.getCurrentPoseEstimation()->getMean(robotPoseTF);

    stamp=ros::Time(0);
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Transform tmp_tf;
    mrpt_bridge::convert(robotPoseTF, tmp_tf);
 
    try
    {
        tf::Stamped<tf::Pose> tmp_tf_stamped (tmp_tf.inverse(), stamp,  base_frame_id);
        listenerTF_.transformPose(odom_frame_id, tmp_tf_stamped, odom_to_map);

    }
    catch(tf::TransformException)
    {
		ROS_INFO("Failed to subtract global_frame (%s) from odom_frame (%s)", global_frame_id.c_str(), odom_frame_id.c_str());
        return;
    }


    tf::Transform latest_tf_ = tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                               tf::Point(odom_to_map.getOrigin()));

 

   tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        stamp,
                                        global_frame_id, odom_frame_id);

   tf_broadcaster_.sendTransform(tmp_tf_stamped);
 
}
