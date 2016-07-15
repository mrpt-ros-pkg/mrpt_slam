/*
 * File: mrpt_ekf_slam_3d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_3d/mrpt_ekf_slam_3d_wrapper.h"


EKFslamWrapper::EKFslamWrapper(){
     rawlog_play_=false;
     mrpt_bridge::convert(ros::Time(0), timeLastUpdate_);
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


     //read sensor topics
    std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source," ,\t\n",lstSources);
	ROS_ASSERT_MSG(!lstSources.empty(), "*Fatal*: At least one sensor source must be provided in ~sensor_sources (e.g. \"scan\" or \"beacon\")");
	

    ///Create subscribers///	
	sensorSub_.resize(lstSources.size());
	for (size_t i=0;i<lstSources.size();i++) {
		if(lstSources[i].find("landmark") != std::string::npos) {
			sensorSub_[i]  = n_.subscribe(lstSources[i],  1, &EKFslamWrapper::landmarkCallback, this);
		}
		else {
           ROS_ERROR("Can't find the sensor topics. The sensor topics should contain the word \"landmark\" in the name");
		}
	}




}

void EKFslamWrapper::odometryForCallback (CObservationOdometryPtr  &_odometry, const std_msgs::Header &_msg_header) {
   
    mrpt::poses::CPose3D poseOdom;
     if(this->waitForTransform(poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp, ros::Duration(1))){
		_odometry = CObservationOdometry::Create();
        _odometry->sensorLabel = odom_frame_id;
        _odometry->hasEncodersInfo = false;
        _odometry->hasVelocities = false;
        _odometry->odometry.x() = poseOdom.x();
        _odometry->odometry.y() = poseOdom.y();
        _odometry->odometry.phi() = poseOdom.yaw();
    }
}

void EKFslamWrapper::updateSensorPose (std::string _frame_id) {
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
        landmark_poses_[_frame_id] = pose;
    }
    catch (tf::TransformException ex) {
        ROS_ERROR("%s",ex.what());
        ros::Duration(1.0).sleep();
    }

}


bool EKFslamWrapper::waitForTransform(mrpt::poses::CPose3D &des, const std::string& target_frame, const std::string& source_frame, const ros::Time& time, const ros::Duration& timeout, const ros::Duration& polling_sleep_duration){
    tf::StampedTransform transform;
    try
    {
        listenerTF_.waitForTransform(target_frame, source_frame,  time, polling_sleep_duration);
        listenerTF_.lookupTransform(target_frame, source_frame,  time, transform);
    }
    catch(tf::TransformException)
    {
        ROS_INFO("Failed to get transform target_frame (%s) to source_frame (%s)", target_frame.c_str(), source_frame.c_str());
        return false;
    }
    mrpt_bridge::convert(transform, des);
    return true;
}

void EKFslamWrapper::landmarkCallback(const mrpt_msgs::ObservationRangeBearing &_msg) {
#if MRPT_VERSION>=0x130
	using namespace mrpt::maps;
	using namespace mrpt::obs;
#else
	using namespace mrpt::slam;
#endif
	CObservationBearingRangePtr landmark = CObservationBearingRange::Create();

    if(landmark_poses_.find(_msg.header.frame_id) == landmark_poses_.end()) { 
        updateSensorPose (_msg.header.frame_id);    
    } else {        
        mrpt::poses::CPose3D pose = landmark_poses_[_msg.header.frame_id];
        mrpt_bridge::convert(_msg, landmark_poses_[_msg.header.frame_id],  *landmark);

	sf = CSensoryFrame::Create();
	CObservationOdometryPtr odometry;
	odometryForCallback(odometry, _msg.header);

	CObservationPtr obs = CObservationPtr(landmark);
        sf->insert(obs);
        observation(sf, odometry);
        timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

                 tictac.Tic();
             mapping.processActionObservation( action,sf);
             t_exec = tictac.Tac();
             printf("Map building executed in %.03fms\n", 1000.0f*t_exec );
             ros::Duration(rawlog_play_delay).sleep();
             mapping.getCurrentState( robotPose_,LMs_,LM_IDs_,fullState_,fullCov_ );
             viz_state(); 

    }
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


void EKFslamWrapper::publishTF() {
     mapping.getCurrentState( robotPose_,LMs_,LM_IDs_,fullState_,fullCov_ );
	// Most of this code was copy and pase from ros::amcl
	mrpt::poses::CPose3D		robotPose;

    robotPose = CPose3D(robotPose_.mean);
	//mapBuilder->mapPDF.getEstimatedPosePDF(curPDF);
  
	//curPDF.getMean(robotPose);
 
    tf::Stamped<tf::Pose> odom_to_map;
    tf::Transform tmp_tf;
    ros::Time stamp;
   mrpt_bridge::convert(timeLastUpdate_, stamp);
    mrpt_bridge::convert(robotPose, tmp_tf);
 
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

    // We want to send a transform that is good up until a
    // tolerance time so that odom can be used

    ros::Duration transform_tolerance_(0.1);
    ros::Time transform_expiration = (stamp + transform_tolerance_);
    tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                        transform_expiration,
                                        global_frame_id, odom_frame_id);
    tf_broadcaster_.sendTransform(tmp_tf_stamped);

}