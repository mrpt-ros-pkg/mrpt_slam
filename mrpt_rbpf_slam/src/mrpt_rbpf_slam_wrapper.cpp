#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt_rbpf_slam/options.h>
using mrpt::maps::CBeaconMap;
using mrpt::maps::COccupancyGridMap2D;

namespace
{
bool isFileExists(const std::string& name)
{
	std::ifstream f(name.c_str());
	return f.good();
}
}  // namespace

namespace mrpt_rbpf_slam
{
PFslamWrapper::PFslamWrapper() { timeLastUpdate_ = mrpt::Clock::now(); }

bool PFslamWrapper::getParams(const ros::NodeHandle& nh_p)
{
	ROS_INFO("READ PARAM FROM LAUNCH FILE");
	nh_p.param<double>("rawlog_play_delay", rawlog_play_delay_, 0.1);
	ROS_INFO("rawlog_play_delay: %f", rawlog_play_delay_);

	nh_p.getParam("rawlog_filename", rawlog_filename_);
	ROS_INFO("rawlog_filename: %s", rawlog_filename_.c_str());

	nh_p.getParam("ini_filename", ini_filename_);
	ROS_INFO("ini_filename: %s", ini_filename_.c_str());

	nh_p.param<std::string>("global_frame_id", global_frame_id_, "map");
	ROS_INFO("global_frame_id: %s", global_frame_id_.c_str());

	nh_p.param<std::string>("odom_frame_id", odom_frame_id_, "odom");
	ROS_INFO("odom_frame_id: %s", odom_frame_id_.c_str());

	nh_p.param<std::string>("base_frame_id", base_frame_id_, "base_link");
	ROS_INFO("base_frame_id: %s", base_frame_id_.c_str());

	nh_p.param<std::string>("sensor_source", sensor_source_, "scan");
	ROS_INFO("sensor_source: %s", sensor_source_.c_str());

	nh_p.param<std::string>("sensor_source", sensor_source_, "scan");
	ROS_INFO("sensor_source: %s", sensor_source_.c_str());

	nh_p.param<bool>("update_sensor_pose", update_sensor_pose_, true);
	ROS_INFO(
		"update_sensor_pose: %s", (update_sensor_pose_ ? "TRUE" : "FALSE"));

	PFslam::Options options;
	if (!loadOptions(nh_p, options))
	{
		ROS_ERROR("Not able to read all parameters!");
		return false;
	}
	initSlam(std::move(options));
	return true;
}

bool PFslamWrapper::init(ros::NodeHandle& nh)
{
	// get parameters from ini file
	if (!isFileExists(ini_filename_))
	{
		ROS_ERROR_STREAM("CAN'T READ INI FILE" << ini_filename_);
		return false;
	}

	PFslam::readIniFile(ini_filename_);

	// read rawlog file if it  exists
	if (isFileExists(rawlog_filename_))
	{
		ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename_);
		PFslam::readRawlog(rawlog_filename_, data_);
		rawlog_play_ = true;
	}

	/// Create publishers///
	// publish grid map
	pub_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
	pub_metadata_ =
		nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
	// robot pose
	pub_particles_ =
		nh.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
	// ro particles poses
	pub_particles_beacons_ = nh.advertise<geometry_msgs::PoseArray>(
		"particlecloud_beacons", 1, true);
	beacon_viz_pub_ =
		nh.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 1);

	// read sensor topics
	std::vector<std::string> lstSources;
	mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
	ROS_ASSERT_MSG(
		!lstSources.empty(),
		"*Fatal*: At least one sensor source must be provided in "
		"~sensor_sources (e.g. "
		"\"scan\" or \"beacon\")");

	/// Create subscribers///
	sensorSub_.resize(lstSources.size());
	for (size_t i = 0; i < lstSources.size(); i++)
	{
		// if (lstSources[i].find("scan") != std::string::npos)
		{
			sensorSub_[i] = nh.subscribe(
				lstSources[i], 1, &PFslamWrapper::laserCallback, this);
		}
#if 0
		else
		{
			sensorSub_[i] = nh.subscribe(
				lstSources[i], 1, &PFslamWrapper::callbackBeacon, this);
		}
#endif
	}

	mapBuilder_ =
		mrpt::slam::CMetricMapBuilderRBPF(options_.rbpfMappingOptions_);
	init3Dwindow();
	return true;
}

void PFslamWrapper::odometryForCallback(
	mrpt::obs::CObservationOdometry::Ptr& odometry,
	const std_msgs::Header& _msg_header)
{
	mrpt::poses::CPose3D poseOdom;
	if (this->waitForTransform(
			poseOdom, odom_frame_id_, base_frame_id_, _msg_header.stamp,
			ros::Duration(1)))
	{
		odometry = mrpt::obs::CObservationOdometry::Create();
		odometry->sensorLabel = odom_frame_id_;
		odometry->hasEncodersInfo = false;
		odometry->hasVelocities = false;
		odometry->odometry.x() = poseOdom.x();
		odometry->odometry.y() = poseOdom.y();
		odometry->odometry.phi() = poseOdom.yaw();
	}
}

bool PFslamWrapper::waitForTransform(
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

void PFslamWrapper::laserCallback(const sensor_msgs::LaserScan& msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;
	CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();

	if (laser_poses_.find(msg.header.frame_id) == laser_poses_.end())
	{
		// check if the tf to establish the sensor pose
		updateSensorPose(msg.header.frame_id);
	}
	else
	{
		// update sensor pose
		if (update_sensor_pose_) updateSensorPose(msg.header.frame_id);
		mrpt::poses::CPose3D pose = laser_poses_[msg.header.frame_id];
		mrpt::ros1bridge::fromROS(
			msg, laser_poses_[msg.header.frame_id], *laser);

		sensory_frame_ = CSensoryFrame::Create();
		CObservationOdometry::Ptr odometry;
		odometryForCallback(odometry, msg.header);

		CObservation::Ptr obs = CObservation::Ptr(laser);
		sensory_frame_->insert(obs);
		observation(sensory_frame_, odometry);
		timeLastUpdate_ = sensory_frame_->getObservationByIndex(0)->timestamp;

		tictac_.Tic();
		mapBuilder_.processActionObservation(*action_, *sensory_frame_);
		t_exec_ = tictac_.Tac();
		ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec_);
		publishMapPose();
		run3Dwindow();
		publishTF();
	}
}

void PFslamWrapper::callbackBeacon(const mrpt_msgs::ObservationRangeBeacon& msg)
{
	using namespace mrpt::maps;
	using namespace mrpt::obs;

	CObservationBeaconRanges::Ptr beacon = CObservationBeaconRanges::Create();

	if (beacon_poses_.find(msg.header.frame_id) == beacon_poses_.end())
	{
		// check if the tf to establish the sensor pose
		updateSensorPose(msg.header.frame_id);
	}
	else
	{
		// update sensor pose
		if (update_sensor_pose_) updateSensorPose(msg.header.frame_id);

		mrpt_msgs_bridge::fromROS(
			msg, beacon_poses_[msg.header.frame_id], *beacon);

		sensory_frame_ = CSensoryFrame::Create();
		CObservationOdometry::Ptr odometry;
		odometryForCallback(odometry, msg.header);

		CObservation::Ptr obs = CObservation::Ptr(beacon);
		sensory_frame_->insert(obs);
		observation(sensory_frame_, odometry);
		timeLastUpdate_ = sensory_frame_->getObservationByIndex(0)->timestamp;

		tictac_.Tic();
		mapBuilder_.processActionObservation(*action_, *sensory_frame_);
		t_exec_ = tictac_.Tac();
		ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec_);

		publishMapPose();
		run3Dwindow();
	}
}

void PFslamWrapper::publishMapPose()
{
	// if I received new grid maps from 2D laser scan sensors
	metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
	mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);
	// publish map

	COccupancyGridMap2D* grid = nullptr;
	CBeaconMap* bm = nullptr;
	if (metric_map_->countMapsByClass<COccupancyGridMap2D>())
		grid = metric_map_->mapByClass<COccupancyGridMap2D>().get();
	bm = metric_map_->mapByClass<CBeaconMap>().get();

	if (grid)
	{
		// publish map
		nav_msgs::OccupancyGrid msg;
		mrpt::ros1bridge::toROS(*grid, msg);
		pub_map_.publish(msg);
		pub_metadata_.publish(msg.info);
	}

	// if I received new beacon (range only) map
	if (bm)
	{
		// Get th map as the set of 3D objects
		const auto objs = bm->getVisualization();

		geometry_msgs::PoseArray poseArrayBeacons;
		poseArrayBeacons.header.frame_id = global_frame_id_;
		poseArrayBeacons.header.stamp = ros::Time::now();

		// Count the number of beacons
		unsigned int objs_counter = 0;
		while (objs->getByClass<mrpt::opengl::CEllipsoid3D>(objs_counter))
		{
			objs_counter++;
		}
		poseArrayBeacons.poses.resize(objs_counter);
		mrpt::opengl::CEllipsoid3D::Ptr beacon_particle;

		for (size_t i = 0; i < objs_counter; i++)
		{
			beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid3D>(i);
			poseArrayBeacons.poses[i] = mrpt::ros1bridge::toROS_Pose(
				mrpt::poses::CPose3D(beacon_particle->getPose()));
			viz_beacons_.push_back(beacon_particle);
		}
		pub_particles_beacons_.publish(poseArrayBeacons);
		vizBeacons();
		viz_beacons_.clear();
	}

	// publish pose
	geometry_msgs::PoseArray poseArray;
	poseArray.header.frame_id = global_frame_id_;
	poseArray.header.stamp = ros::Time::now();
	poseArray.poses.resize(curPDF.particlesCount());
	for (size_t i = 0; i < curPDF.particlesCount(); i++)
	{
		const auto p = mrpt::poses::CPose3D(curPDF.getParticlePose(i));
		poseArray.poses[i] = mrpt::ros1bridge::toROS_Pose(p);
	}

	pub_particles_.publish(poseArray);
}

void PFslamWrapper::vizBeacons()
{
	if (viz_beacons_.size() == 0)
	{
		return;
	}
	visualization_msgs::MarkerArray ma;
	visualization_msgs::Marker marker;

	marker.header.frame_id = "map";

	marker.id = 0;
	marker.type = visualization_msgs::Marker::SPHERE;
	marker.action = visualization_msgs::Marker::ADD;
	marker.lifetime = ros::Duration(1);
	marker.pose.position.x = 0;
	marker.pose.position.y = 0;
	marker.pose.position.z = 0;
	marker.pose.orientation.x = 0.0;
	marker.pose.orientation.y = 0.0;
	marker.pose.orientation.z = 0.0;
	marker.pose.orientation.w = 1.0;
	marker.scale.x = 0.12;
	marker.scale.y = 0.12;
	marker.scale.z = 0.12;
	marker.color.a = 1.0;
	marker.color.r = 1.0;
	marker.color.g = 0.0;

	for (unsigned int i = 0; i < viz_beacons_.size(); i++)
	{
		mrpt::poses::CPose3D meanPose(viz_beacons_[i]->getPose());
		marker.type = visualization_msgs::Marker::SPHERE;

		marker.pose.position.x = meanPose.x();
		marker.pose.position.y = meanPose.y();
		marker.pose.position.z = meanPose.z();
		marker.color.r = 1.0;
		marker.color.g = 0.0;
		marker.color.b = 0.0;

		ma.markers.push_back(marker);
		marker.id++;

		marker.type = visualization_msgs::Marker::TEXT_VIEW_FACING;
		marker.text = std::to_string(i);

		marker.pose.position.x = meanPose.x();
		marker.pose.position.y = meanPose.y();
		marker.pose.position.z = meanPose.z() + 0.12;
		marker.color.r = 1.0;
		marker.color.g = 1.0;
		marker.color.b = 1.0;
		// marker.scale.z = 1;
		ma.markers.push_back(marker);
		marker.id++;
	}

	beacon_viz_pub_.publish(ma);
}

void PFslamWrapper::updateSensorPose(const std::string& frame_id)
{
	geometry_msgs::TransformStamped transformStmp;
	try
	{
		ros::Duration timeout(1.0);
		transformStmp = tf_buffer_.lookupTransform(
			base_frame_id_, frame_id, ros::Time(0), timeout);
	}
	catch (const tf2::TransformException& e)
	{
		ROS_WARN(
			"Failed to get transform target_frame (%s) to source_frame (%s): "
			"%s",
			base_frame_id_.c_str(), frame_id.c_str(), e.what());
		return;
	}
	tf2::Transform transform;
	tf2::fromMsg(transformStmp.transform, transform);
	const mrpt::poses::CPose3D pose = mrpt::ros1bridge::fromROS(transform);

	laser_poses_[frame_id] = pose;
	beacon_poses_[frame_id] = pose;
}

bool PFslamWrapper::rawlogPlay()
{
	if (rawlog_play_ == false)
	{
		return false;
	}
	else
	{
		for (unsigned int i = 0; i < data_.size(); i++)
		{
			if (ros::ok())
			{
				tictac_.Tic();
				mapBuilder_.processActionObservation(
					data_[i].first, data_[i].second);
				t_exec_ = tictac_.Tac();
				ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec_);

				ros::Duration(rawlog_play_delay_).sleep();

				metric_map_ =
					mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
				mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);

				COccupancyGridMap2D* grid = nullptr;
				CBeaconMap* bm = nullptr;
				if (metric_map_->countMapsByClass<COccupancyGridMap2D>())
					grid = metric_map_->mapByClass<COccupancyGridMap2D>().get();
				bm = metric_map_->mapByClass<CBeaconMap>().get();

				// if I received new grid maps from 2D laser scan sensors
				if (grid)
				{
					nav_msgs::OccupancyGrid msg;
					// if we have new map for current sensor update it
					mrpt::ros1bridge::toROS(*grid, msg);
					pub_map_.publish(msg);
					pub_metadata_.publish(msg.info);
				}

				// if I received new beacon (range only) map
				if (bm)
				{
					const auto objs = bm->getVisualization();

					geometry_msgs::PoseArray poseArrayBeacons;
					poseArrayBeacons.header.frame_id = global_frame_id_;
					poseArrayBeacons.header.stamp = ros::Time::now();

					unsigned int objs_counter = 0;
					while (objs->getByClass<mrpt::opengl::CEllipsoid3D>(
						objs_counter))
					{
						objs_counter++;
					}
					poseArrayBeacons.poses.resize(objs_counter);
					mrpt::opengl::CEllipsoid3D::Ptr beacon_particle;

					for (size_t i = 0; i < objs_counter; i++)
					{
						beacon_particle =
							objs->getByClass<mrpt::opengl::CEllipsoid3D>(i);
						poseArrayBeacons.poses[i] =
							mrpt::ros1bridge::toROS_Pose(mrpt::poses::CPose3D(
								beacon_particle->getPose()));
						viz_beacons_.push_back(beacon_particle);
					}
					pub_particles_beacons_.publish(poseArrayBeacons);
					vizBeacons();
					viz_beacons_.clear();
				}

				// publish pose
				geometry_msgs::PoseArray poseArray;
				poseArray.header.frame_id = global_frame_id_;
				poseArray.header.stamp = ros::Time::now();
				poseArray.poses.resize(curPDF.particlesCount());
				for (size_t i = 0; i < curPDF.particlesCount(); i++)
				{
					const auto p =
						mrpt::poses::CPose3D(curPDF.getParticlePose(i));
					poseArray.poses[i] = mrpt::ros1bridge::toROS_Pose(p);
				}

				pub_particles_.publish(poseArray);
			}
			ros::spinOnce();
			run3Dwindow();
		}
	}
	// if there is mrpt_gui it will wait until push any key in order to close
	// the window
	if (win3D_) win3D_->waitForKey();
	return true;
}

void PFslamWrapper::publishTF()
{
	const ros::Time stamp = mrpt::ros1bridge::toROS(timeLastUpdate_);

	// Most of this code was copy and pase from ros::amcl
	mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);
	const mrpt::poses::CPose3D robotPoseTF = curPDF.getMeanVal();

	geometry_msgs::PoseStamped odom_to_map;

	try
	{
		tf2::Transform tmp_tf =
			mrpt::ros1bridge::toROS_tfTransform(robotPoseTF);

		geometry_msgs::PoseStamped tmp_tf_stamped;
		tmp_tf_stamped.header.frame_id = base_frame_id_;
		tmp_tf_stamped.header.stamp = stamp;
		tf2::toMsg(tmp_tf.inverse(), tmp_tf_stamped.pose);

		tf_buffer_.transform(tmp_tf_stamped, odom_to_map, odom_frame_id_);
	}
	catch (const tf2::TransformException&)
	{
		ROS_INFO(
			"Failed to subtract global_frame (%s) from odom_frame (%s)",
			global_frame_id_.c_str(), odom_frame_id_.c_str());
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
		tmp_tf_stamped.header.frame_id = global_frame_id_;
		tmp_tf_stamped.header.stamp = transform_expiration;
		tmp_tf_stamped.child_frame_id = odom_frame_id_;
		tf2::convert(latest_tf.inverse(), tmp_tf_stamped.transform);

		tf_broadcaster_.sendTransform(tmp_tf_stamped);
	}
}

}  // namespace mrpt_rbpf_slam
