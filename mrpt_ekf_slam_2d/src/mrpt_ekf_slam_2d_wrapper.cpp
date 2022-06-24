/*
 * File: mrpt_ekf_slam_2d_wrapper.cpp
 * Author: Vladislav Tananaev
 *
 */

#include "mrpt_ekf_slam_2d/mrpt_ekf_slam_2d_wrapper.h"

#include <mrpt/serialization/CArchive.h>

EKFslamWrapper::EKFslamWrapper()
{
	rawlog_play_ = false;
	timeLastUpdate_ = mrpt::Clock::now();
}
EKFslamWrapper::~EKFslamWrapper() {}
bool EKFslamWrapper::is_file_exists(const std::string& name)
{
	std::ifstream f(name.c_str());
	return f.good();
}

void EKFslamWrapper::get_param()
{
	ROS_INFO("READ PARAM FROM LAUNCH FILE");
	n_.param<double>("ellipse_scale", ellipse_scale_, 1);
	ROS_INFO("ellipse_scale: %f", ellipse_scale_);

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
void EKFslamWrapper::init()
{
	// get parameters from ini file

	if (!is_file_exists(ini_filename))
	{
		ROS_ERROR_STREAM("CAN'T READ INI FILE");
		return;
	}

	EKFslam::read_iniFile(ini_filename);
	// read rawlog file if it  exists
	if (is_file_exists(rawlog_filename))
	{
		ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename.c_str());
		rawlog_play_ = true;
	}

	state_viz_pub_ =
		n_.advertise<visualization_msgs::MarkerArray>("/state_viz", 1);	 // map
	data_association_viz_pub_ = n_.advertise<visualization_msgs::MarkerArray>(
		"/data_association_viz", 1);  // data_association

	// read sensor topics
	std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source, " ,\t\n", lstSources);
	ROS_ASSERT_MSG(
		!lstSources.empty(),
		"*Fatal*: At least one sensor source must be provided in "
		"~sensor_sources (e.g. "
		"\"scan\" or \"beacon\")");

	/// Create subscribers///
	sensorSub_.resize(lstSources.size());
	for (size_t i = 0; i < lstSources.size(); i++)
	{
		if (lstSources[i].find("landmark") != std::string::npos)
		{
			sensorSub_[i] = n_.subscribe(
				lstSources[i], 1, &EKFslamWrapper::landmarkCallback, this);
		}
		else
		{
			ROS_ERROR(
				"Can't find the sensor topics. The sensor topics should "
				"contain the word \"landmark\" in the name");
		}
	}

	init3Dwindow();
}

void EKFslamWrapper::odometryForCallback(
	mrpt::obs::CObservationOdometry::Ptr& _odometry,
	const std_msgs::Header& _msg_header)
{
	mrpt::poses::CPose3D poseOdom;
	if (this->waitForTransform(
			poseOdom, odom_frame_id, base_frame_id, _msg_header.stamp,
			ros::Duration(1)))
	{
		_odometry = mrpt::obs::CObservationOdometry::Create();
		_odometry->sensorLabel = odom_frame_id;
		_odometry->hasEncodersInfo = false;
		_odometry->hasVelocities = false;
		_odometry->odometry.x() = poseOdom.x();
		_odometry->odometry.y() = poseOdom.y();
		_odometry->odometry.phi() = poseOdom.yaw();
	}
}

void EKFslamWrapper::updateSensorPose(std::string _frame_id)
{
	geometry_msgs::TransformStamped transformStmp;
	try
	{
		ros::Duration timeout(1.0);
		transformStmp = tf_buffer_.lookupTransform(
			base_frame_id, _frame_id, ros::Time(0), timeout);
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			base_frame_id.c_str(), _frame_id.c_str(), e.what());
		return;
	}
	tf2::Transform transform;
	tf2::fromMsg(transformStmp.transform, transform);
	const mrpt::poses::CPose3D pose = mrpt::ros1bridge::fromROS(transform);

	landmark_poses_[_frame_id] = pose;
}

bool EKFslamWrapper::waitForTransform(
	mrpt::poses::CPose3D& des, const std::string& target_frame,
	const std::string& source_frame, const ros::Time& time,
	const ros::Duration& timeout, const ros::Duration& polling_sleep_duration)
{
	geometry_msgs::TransformStamped transform;
	try
	{
		transform = tf_buffer_.lookupTransform(
			target_frame, source_frame, time, timeout);
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			target_frame.c_str(), source_frame.c_str(), e.what());
		return false;
	}
	tf2::Transform tx;
	tf2::fromMsg(transform.transform, tx);
	des = mrpt::ros1bridge::fromROS(tx);
	return true;
}

void EKFslamWrapper::landmarkCallback(
	const mrpt_msgs::ObservationRangeBearing& _msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	CObservationBearingRange::Ptr landmark = CObservationBearingRange::Create();

	if (landmark_poses_.find(_msg.header.frame_id) == landmark_poses_.end())
	{
		updateSensorPose(_msg.header.frame_id);
	}
	else
	{
		mrpt::poses::CPose3D pose = landmark_poses_[_msg.header.frame_id];
		mrpt_msgs_bridge::fromROS(
			_msg, landmark_poses_[_msg.header.frame_id], *landmark);

		sf = CSensoryFrame::Create();
		CObservationOdometry::Ptr odometry;
		odometryForCallback(odometry, _msg.header);

		CObservation::Ptr obs = CObservation::Ptr(landmark);
		sf->insert(obs);
		observation(sf, odometry);
		timeLastUpdate_ = sf->getObservationByIndex(0)->timestamp;

		tictac.Tic();
		mapping.processActionObservation(action, sf);
		t_exec = tictac.Tac();
		ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);
		ros::Duration(rawlog_play_delay).sleep();
		mapping.getCurrentState(
			robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);
		viz_state();
		viz_dataAssociation();
		run3Dwindow();
		publishTF();
	}
}

bool EKFslamWrapper::rawlogPlay()
{
	if (rawlog_play_ == false)
	{
		return false;
	}
	else
	{
		size_t rawlogEntry = 0;
		mrpt::io::CFileGZInputStream rawlog_stream(rawlog_filename);
		auto rawlogFile = mrpt::serialization::archiveFrom(rawlog_stream);

		mrpt::obs::CActionCollection::Ptr action;
		mrpt::obs::CSensoryFrame::Ptr observations;

		for (;;)
		{
			if (ros::ok())
			{
				if (!mrpt::obs::CRawlog::readActionObservationPair(
						rawlogFile, action, observations, rawlogEntry))
				{
					break;	// file EOF
				}
				tictac.Tic();
				mapping.processActionObservation(action, observations);
				t_exec = tictac.Tac();
				ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec);
				ros::Duration(rawlog_play_delay).sleep();
				mapping.getCurrentState(
					robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);

				viz_state();
				viz_dataAssociation();
				run3Dwindow();
			}
		}
		if (win3d)
		{
			std::cout << "\n Close the 3D window to quit the application.\n";
			win3d->waitForKey();
		}
		return true;
	}
}

// Local function to force the axis to be right handed for 2D. Based on the one
// from ecl_statistics
void EKFslamWrapper::makeRightHanded(
	Eigen::Matrix2d& eigenvectors, Eigen::Vector2d& eigenvalues)
{
	// Note that sorting of eigenvalues may end up with left-hand coordinate
	// system. So here we correctly sort it so that it does end up being
	// righ-handed and normalised.
	Eigen::Vector3d c0;
	c0.setZero();
	c0.head<2>() = eigenvectors.col(0);
	c0.normalize();
	Eigen::Vector3d c1;
	c1.setZero();
	c1.head<2>() = eigenvectors.col(1);
	c1.normalize();
	Eigen::Vector3d cc = c0.cross(c1);
	if (cc[2] < 0)
	{
		eigenvectors << c1.head<2>(), c0.head<2>();
		double e = eigenvalues[0];
		eigenvalues[0] = eigenvalues[1];
		eigenvalues[1] = e;
	}
	else
	{
		eigenvectors << c0.head<2>(), c1.head<2>();
	}
}

void EKFslamWrapper::computeEllipseOrientationScale2D(
	tf2::Quaternion& orientation, Eigen::Vector2d& scale,
	const mrpt::math::CMatrixDouble22& covariance)
{
	tf2::Matrix3x3 tf3d;
	Eigen::Vector2d eigenvalues(Eigen::Vector2d::Identity());
	Eigen::Matrix2d eigenvectors(Eigen::Matrix2d::Zero());

	// NOTE: The SelfAdjointEigenSolver only references the lower triangular
	// part of the covariance matrix
	Eigen::SelfAdjointEigenSolver<Eigen::Matrix2d> eigensolver(
		covariance.asEigen());

	// Compute eigenvectors and eigenvalues
	if (eigensolver.info() == Eigen::Success)
	{
		eigenvalues = eigensolver.eigenvalues();
		eigenvectors = eigensolver.eigenvectors();
	}
	else
	{
		ROS_ERROR_STREAM(
			"failed to compute eigen vectors/values for position. Is "
			"the covariance matrix correct?");
		eigenvalues = Eigen::Vector2d::Zero();	// Setting the scale to zero
												// will hide it on the screen
		eigenvectors = Eigen::Matrix2d::Identity();
	}

	// Be sure we have a right-handed orientation system
	makeRightHanded(eigenvectors, eigenvalues);

	// Rotation matrix around  z axis
	tf3d.setValue(
		eigenvectors(0, 0), eigenvectors(0, 1), 0, eigenvectors(1, 0),
		eigenvectors(1, 1), 0, 0, 0, 1);

	// get orientation from rotation matrix
	tf3d.getRotation(orientation);
	// get scale
	scale[0] = eigenvalues[0];
	scale[1] = eigenvalues[1];
}

void EKFslamWrapper::viz_dataAssociation()
{
	// robot pose
	mrpt::poses::CPose3D robotPose;
	robotPose = mrpt::poses::CPose3D(robotPose_.mean);
	geometry_msgs::Point pointRobotPose;
	pointRobotPose.z = 0;
	pointRobotPose.x = robotPose.x();
	pointRobotPose.y = robotPose.y();

	// visualization of the data association
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker line_strip;

	line_strip.header.frame_id = "map";
	line_strip.header.stamp = ros::Time::now();

	line_strip.id = 0;
	line_strip.type = visualization_msgs::Marker::LINE_STRIP;
	line_strip.action = visualization_msgs::Marker::ADD;

	line_strip.lifetime = ros::Duration(0.1);
	line_strip.pose.position.x = 0;
	line_strip.pose.position.y = 0;
	line_strip.pose.position.z = 0;
	line_strip.pose.orientation.x = 0.0;
	line_strip.pose.orientation.y = 0.0;
	line_strip.pose.orientation.z = 0.0;
	line_strip.pose.orientation.w = 1.0;
	line_strip.scale.x = 0.02;	// line uses only x component
	line_strip.scale.y = 0.0;
	line_strip.scale.z = 0.0;
	line_strip.color.a = 1.0;
	line_strip.color.r = 1.0;
	line_strip.color.g = 1.0;
	line_strip.color.b = 1.0;

	// Draw latest data association:
	const CRangeBearingKFSLAM2D::TDataAssocInfo& da =
		mapping.getLastDataAssociation();

	for (auto it = da.results.associations.begin();
		 it != da.results.associations.end(); ++it)
	{
		const mrpt::slam::prediction_index_t idxPred = it->second;

		// This index must match the internal list of features in the map:
		CRangeBearingKFSLAM2D::KFArray_FEAT featMean;
		mapping.getLandmarkMean(idxPred, featMean);

		line_strip.points.clear();
		line_strip.points.push_back(pointRobotPose);
		geometry_msgs::Point pointLm;
		pointLm.z = 0.0;
		pointLm.x = featMean[0];
		pointLm.y = featMean[1];
		line_strip.points.push_back(pointLm);
		ma.markers.push_back(line_strip);
		line_strip.id++;
	}

	data_association_viz_pub_.publish(ma);
}
void EKFslamWrapper::viz_state()
{
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker marker;
	marker.header.frame_id = "map";
	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(0);

	// get the covariance matrix 2x2 for each ellipsoid including robot pose
	mrpt::opengl::CSetOfObjects::Ptr objs;
	objs = mrpt::opengl::CSetOfObjects::Create();
	mapping.getAs3DObject(objs);

	// Count the number of landmarks
	unsigned int objs_counter = 0;
	while (objs->getByClass<mrpt::opengl::CEllipsoid2D>(objs_counter))
	{
		objs_counter++;
	}

	mrpt::opengl::CEllipsoid2D::Ptr landmark;
	for (size_t i = 0; i < objs_counter; i++)
	{
		landmark = objs->getByClass<mrpt::opengl::CEllipsoid2D>(i);

		const auto covariance = landmark->getCovMatrix();
		float quantiles = landmark->getQuantiles();

		// mean position
		// pose of the robot and landmarks (x,y,z=0)
		const auto pose = mrpt::poses::CPose3D(landmark->getPose());

		// covariance ellipses
		tf2::Quaternion orientation;
		Eigen::Vector2d scale;

		computeEllipseOrientationScale2D(orientation, scale, covariance);

		marker.id++;
		marker.color.a = 1.0;
		if (i == 0)
		{  // robot position
			marker.color.r = 1.0;
			marker.color.g = 0.0;
			marker.color.b = 0.0;
		}
		else
		{
			marker.color.r = 0.0;
			marker.color.g = 0.0;
			marker.color.b = 1.0;
		}
		marker.type = visualization_msgs::Marker::SPHERE;
		marker.pose.position.x = pose.x();
		marker.pose.position.y = pose.y();
		marker.pose.position.z = 0;
		marker.pose.orientation.x = orientation.x();
		marker.pose.orientation.y = orientation.y();
		marker.pose.orientation.z = orientation.z();
		marker.pose.orientation.w = orientation.w();
		marker.scale.x = ellipse_scale_ * quantiles * sqrt(scale[0]);
		marker.scale.y = ellipse_scale_ * quantiles * sqrt(scale[1]);
		marker.scale.z = 0.00001;  // Z can't be 0, limitation of ROS
		ma.markers.push_back(marker);

		marker.id++;
		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		if (i == 0)
		{
			marker.text = "robot";
		}
		else
		{
			marker.text = std::to_string(LM_IDs_[i]);
		}
		marker.pose.position.x = pose.x();
		marker.pose.position.y = pose.y();
		marker.pose.position.z = 0.1;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		marker.scale.x = 0.3;
		marker.scale.y = 0.3;
		marker.scale.z = 0.3;
		ma.markers.push_back(marker);
	}

	state_viz_pub_.publish(ma);
}

void EKFslamWrapper::publishTF()
{
	mapping.getCurrentState(robotPose_, LMs_, LM_IDs_, fullState_, fullCov_);

	// Most of this code was copy and pase from ros::amcl
	const mrpt::poses::CPose3D robotPoseTF =
		mrpt::poses::CPose3D(robotPose_.mean);

	const ros::Time stamp = mrpt::ros1bridge::toROS(timeLastUpdate_);

	geometry_msgs::PoseStamped odom_to_map;

	try
	{
		tf2::Transform tmp_tf =
			mrpt::ros1bridge::toROS_tfTransform(robotPoseTF);

		geometry_msgs::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = base_frame_id;
		tmp_tf_stamped.header.stamp = stamp;
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

		tf_buffer_.transform(tmp_tf_stamped, odom_to_map, odom_frame_id);
	}
	catch (const tf2::TransformException&)
	{
		ROS_INFO(
			"Failed to subtract global_frame (%s) from odom_frame (%s)",
			global_frame_id.c_str(), odom_frame_id.c_str());
		return;
	}

	{
		tf2::Transform latest_tf;
		tf2::convert(odom_to_map.pose, latest_tf);

		// We want to send a transform that is good up until a
		// tolerance time so that odom can be used
		ros::Duration transform_tolerance(0.1);

		ros::Time transform_expiration = stamp + transform_tolerance;

		geometry_msgs::TransformStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = global_frame_id;
		tmp_tf_stamped.header.stamp = transform_expiration;
		tmp_tf_stamped.child_frame_id = odom_frame_id;
		tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);

		tf_broadcaster_.sendTransform(tmp_tf_stamped);
	}
}
