#include "mrpt_rbpf_slam/mrpt_rbpf_slam_wrapper.h"
#include <mrpt/maps/CBeaconMap.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt_rbpf_slam/options.h>
using mrpt::maps::CBeaconMap;
using mrpt::maps::COccupancyGridMap2D;

namespace {
bool isFileExists(const std::string &name) {
  std::ifstream f(name.c_str());
  return f.good();
}
} // namespace

namespace mrpt_rbpf_slam {
PFslamWrapper::PFslamWrapper() {
  mrpt_bridge::convert(ros::Time(0), timeLastUpdate_);
}

bool PFslamWrapper::getParams(const ros::NodeHandle &nh_p) {
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

  PFslam::Options options;
  if (!loadOptions(nh_p, options)) {
    ROS_ERROR("Not able to read all parameters!");
    return false;
  }
  initSlam(std::move(options));
  return true;
}

bool PFslamWrapper::init(ros::NodeHandle &nh) {
  // get parameters from ini file
  if (!isFileExists(ini_filename_)) {
    ROS_ERROR_STREAM("CAN'T READ INI FILE" << ini_filename_);
    return false;
  }

  PFslam::readIniFile(ini_filename_);

  // read rawlog file if it  exists
  if (isFileExists(rawlog_filename_)) {
    ROS_WARN_STREAM("PLAY FROM RAWLOG FILE: " << rawlog_filename_);
    PFslam::readRawlog(rawlog_filename_, data_);
    rawlog_play_ = true;
  }

  /// Create publishers///
  // publish grid map
  pub_map_ = nh.advertise<nav_msgs::OccupancyGrid>("map", 1, true);
  pub_metadata_ = nh.advertise<nav_msgs::MapMetaData>("map_metadata", 1, true);
  // robot pose
  pub_particles_ =
      nh.advertise<geometry_msgs::PoseArray>("particlecloud", 1, true);
  // ro particles poses
  pub_particles_beacons_ =
      nh.advertise<geometry_msgs::PoseArray>("particlecloud_beacons", 1, true);
  beacon_viz_pub_ =
      nh.advertise<visualization_msgs::MarkerArray>("/beacons_viz", 1);

  // read sensor topics
  std::vector<std::string> lstSources;
  mrpt::system::tokenize(sensor_source_, " ,\t\n", lstSources);
  ROS_ASSERT_MSG(!lstSources.empty(),
                 "*Fatal*: At least one sensor source must be provided in "
                 "~sensor_sources (e.g. "
                 "\"scan\" or \"beacon\")");

  /// Create subscribers///
  sensorSub_.resize(lstSources.size());
  for (size_t i = 0; i < lstSources.size(); i++) {
    if (lstSources[i].find("scan") != std::string::npos) {
      sensorSub_[i] =
          nh.subscribe(lstSources[i], 1, &PFslamWrapper::laserCallback, this);
    } else {
      sensorSub_[i] =
          nh.subscribe(lstSources[i], 1, &PFslamWrapper::callbackBeacon, this);
    }
  }

  mapBuilder_ = mrpt::slam::CMetricMapBuilderRBPF(options_.rbpfMappingOptions_);
  init3Dwindow();
  return true;
}

void PFslamWrapper::odometryForCallback(
    mrpt::obs::CObservationOdometry::Ptr &odometry,
    const std_msgs::Header &_msg_header) {
  mrpt::poses::CPose3D poseOdom;
  if (this->waitForTransform(poseOdom, odom_frame_id_, base_frame_id_,
                             _msg_header.stamp, ros::Duration(1))) {
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
    mrpt::poses::CPose3D &des, const std::string &target_frame,
    const std::string &source_frame, const ros::Time &time,
    const ros::Duration &timeout, const ros::Duration &polling_sleep_duration) {
  tf::StampedTransform transform;
  try {
    listenerTF_.waitForTransform(target_frame, source_frame, time, timeout,
                                 polling_sleep_duration);
    listenerTF_.lookupTransform(target_frame, source_frame, time, transform);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Failed to get transform target_frame (%s) to source_frame (%s). "
              "TransformException: %s",
              target_frame.c_str(), source_frame.c_str(), ex.what());
    return false;
  }
  mrpt_bridge::convert(transform, des);
  return true;
}

void PFslamWrapper::laserCallback(const sensor_msgs::LaserScan &msg) {
  using namespace mrpt::maps;
  using namespace mrpt::obs;
  CObservation2DRangeScan::Ptr laser = CObservation2DRangeScan::Create();

  if (laser_poses_.find(msg.header.frame_id) == laser_poses_.end()) {
    updateSensorPose(msg.header.frame_id);
  } else {
    mrpt::poses::CPose3D pose = laser_poses_[msg.header.frame_id];
    mrpt_bridge::convert(msg, laser_poses_[msg.header.frame_id], *laser);

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

void PFslamWrapper::callbackBeacon(
    const mrpt_msgs::ObservationRangeBeacon &msg) {
  using namespace mrpt::maps;
  using namespace mrpt::obs;

  CObservationBeaconRanges::Ptr beacon = CObservationBeaconRanges::Create();
  if (beacon_poses_.find(msg.header.frame_id) == beacon_poses_.end()) {
    updateSensorPose(msg.header.frame_id);
  } else {
    mrpt_bridge::convert(msg, beacon_poses_[msg.header.frame_id], *beacon);

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

void PFslamWrapper::publishMapPose() {
  // if I received new grid maps from 2D laser scan sensors
  metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
  mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);
  // publish map

  COccupancyGridMap2D *grid = nullptr;
  CBeaconMap *bm = nullptr;
#if MRPT_VERSION >= 0x199
  if (metric_map_->countMapsByClass<COccupancyGridMap2D>())
    grid = metric_map_->mapByClass<COccupancyGridMap2D>().get();
  bm = metric_map_->mapByClass<CBeaconMap>().get();
#else
  if (metric_map_->m_gridMaps.size())
    grid = metric_map_->m_gridMaps[0].get();
  bm = metric_map_->getMapByClass<CBeaconMap>().get();
#endif

  if (grid) {
    // publish map
    nav_msgs::OccupancyGrid msg;
    mrpt_bridge::convert(*grid, msg);
    pub_map_.publish(msg);
    pub_metadata_.publish(msg.info);
  }

  // if I received new beacon (range only) map
  if (bm) {
    mrpt::opengl::CSetOfObjects::Ptr objs;

    objs = mrpt::opengl::CSetOfObjects::Create();
    // Get th map as the set of 3D objects
    bm->getAs3DObject(objs);

    geometry_msgs::PoseArray poseArrayBeacons;
    poseArrayBeacons.header.frame_id = global_frame_id_;
    poseArrayBeacons.header.stamp = ros::Time::now();

    // Count the number of beacons
    unsigned int objs_counter = 0;
    while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter)) {
      objs_counter++;
    }
    poseArrayBeacons.poses.resize(objs_counter);
    mrpt::opengl::CEllipsoid::Ptr beacon_particle;

    for (size_t i = 0; i < objs_counter; i++) {
      beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
      mrpt_bridge::convert(mrpt::poses::CPose3D(beacon_particle->getPose()),
                           poseArrayBeacons.poses[i]);
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
  for (size_t i = 0; i < curPDF.particlesCount(); i++) {
    const auto p = mrpt::poses::CPose3D(curPDF.getParticlePose(i));
    mrpt_bridge::convert(p, poseArray.poses[i]);
  }

  pub_particles_.publish(poseArray);
}

void PFslamWrapper::vizBeacons() {
  if (viz_beacons_.size() == 0) {
    return;
  }
  visualization_msgs::MarkerArray ma;
  visualization_msgs::Marker marker;

  marker.header.frame_id = "/map";

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

  for (unsigned int i = 0; i < viz_beacons_.size(); i++) {
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

void PFslamWrapper::updateSensorPose(const std::string &frame_id) {
  mrpt::poses::CPose3D pose;
  tf::StampedTransform transform;
  try {
    listenerTF_.lookupTransform(base_frame_id_, frame_id, ros::Time(0),
                                transform);

    tf::Vector3 translation = transform.getOrigin();
    tf::Quaternion quat = transform.getRotation();
    pose.x() = translation.x();
    pose.y() = translation.y();
    pose.z() = translation.z();
    tf::Matrix3x3 Rsrc(quat);
    mrpt::math::CMatrixDouble33 Rdes;
    for (int c = 0; c < 3; c++)
      for (int r = 0; r < 3; r++)
        Rdes(r, c) = Rsrc.getRow(r)[c];
    pose.setRotationMatrix(Rdes);
    laser_poses_[frame_id] = pose;
    beacon_poses_[frame_id] = pose;
  } catch (tf::TransformException ex) {
    ROS_ERROR("%s", ex.what());
    ros::Duration(1.0).sleep();
  }
}

bool PFslamWrapper::rawlogPlay() {
  if (rawlog_play_ == false) {
    return false;
  } else {
    for (unsigned int i = 0; i < data_.size(); i++) {
      if (ros::ok()) {
        tictac_.Tic();
        mapBuilder_.processActionObservation(data_[i].first, data_[i].second);
        t_exec_ = tictac_.Tac();
        ROS_INFO("Map building executed in %.03fms", 1000.0f * t_exec_);

        ros::Duration(rawlog_play_delay_).sleep();

        metric_map_ = mapBuilder_.mapPDF.getCurrentMostLikelyMetricMap();
        mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);

        COccupancyGridMap2D *grid = nullptr;
        CBeaconMap *bm = nullptr;
#if MRPT_VERSION >= 0x199
        if (metric_map_->countMapsByClass<COccupancyGridMap2D>())
          grid = metric_map_->mapByClass<COccupancyGridMap2D>().get();
        bm = metric_map_->mapByClass<CBeaconMap>().get();
#else
        if (metric_map_->m_gridMaps.size())
          grid = metric_map_->m_gridMaps[0].get();
        bm = metric_map_->getMapByClass<CBeaconMap>().get();
#endif

        // if I received new grid maps from 2D laser scan sensors
        if (grid) {
          nav_msgs::OccupancyGrid msg;
          // if we have new map for current sensor update it
          mrpt_bridge::convert(*grid, msg);
          pub_map_.publish(msg);
          pub_metadata_.publish(msg.info);
        }

        // if I received new beacon (range only) map
        if (bm) {
          mrpt::opengl::CSetOfObjects::Ptr objs;
          objs = mrpt::opengl::CSetOfObjects::Create();
          bm->getAs3DObject(objs);

          geometry_msgs::PoseArray poseArrayBeacons;
          poseArrayBeacons.header.frame_id = global_frame_id_;
          poseArrayBeacons.header.stamp = ros::Time::now();

          unsigned int objs_counter = 0;
          while (objs->getByClass<mrpt::opengl::CEllipsoid>(objs_counter)) {
            objs_counter++;
          }
          poseArrayBeacons.poses.resize(objs_counter);
          mrpt::opengl::CEllipsoid::Ptr beacon_particle;

          for (size_t i = 0; i < objs_counter; i++) {
            beacon_particle = objs->getByClass<mrpt::opengl::CEllipsoid>(i);
            mrpt_bridge::convert(
                mrpt::poses::CPose3D(beacon_particle->getPose()),
                poseArrayBeacons.poses[i]);
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
        for (size_t i = 0; i < curPDF.particlesCount(); i++) {
          const auto p = mrpt::poses::CPose3D(curPDF.getParticlePose(i));
          mrpt_bridge::convert(p, poseArray.poses[i]);
        }

        pub_particles_.publish(poseArray);
      }
      ros::spinOnce();
      run3Dwindow();
    }
  }
  // if there is mrpt_gui it will wait until push any key in order to close the
  // window
  if (win3D_)
    win3D_->waitForKey();
  return true;
}

void PFslamWrapper::publishTF() {
  // Most of this code was copy and pase from ros::amcl
  mrpt::poses::CPose3D robotPose;
  mapBuilder_.mapPDF.getEstimatedPosePDF(curPDF);

  curPDF.getMean(robotPose);

  tf::Stamped<tf::Pose> odom_to_map;
  tf::Transform tmp_tf;
  ros::Time stamp;
  mrpt_bridge::convert(timeLastUpdate_, stamp);
  mrpt_bridge::convert(robotPose, tmp_tf);

  try {
    tf::Stamped<tf::Pose> tmp_tf_stamped(tmp_tf.inverse(), stamp,
                                         base_frame_id_);
    listenerTF_.transformPose(odom_frame_id_, tmp_tf_stamped, odom_to_map);
  } catch (tf::TransformException ex) {
    ROS_ERROR("Failed to subtract global_frame (%s) from odom_frame (%s). "
              "TransformException: %s",
              global_frame_id_.c_str(), odom_frame_id_.c_str(), ex.what());
    return;
  }

  tf::Transform latest_tf_ =
      tf::Transform(tf::Quaternion(odom_to_map.getRotation()),
                    tf::Point(odom_to_map.getOrigin()));

  // We want to send a transform that is good up until a
  // tolerance time so that odom can be used

  ros::Duration transform_tolerance_(0.5);
  ros::Time transform_expiration = (stamp + transform_tolerance_);
  tf::StampedTransform tmp_tf_stamped(latest_tf_.inverse(),
                                      transform_expiration, global_frame_id_,
                                      odom_frame_id_);
  tf_broadcaster_.sendTransform(tmp_tf_stamped);
}

} // namespace mrpt_rbpf_slam
