/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+
 */

#pragma once

// ROS
#include <ros/ros.h>
#include <mrpt_msgs/GraphSlamStats.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <geometry_msgs/TransformStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Path.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <std_msgs/Header.h>

#include <tf2_ros/transform_broadcaster.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <geometry_msgs/TransformStamped.h>
#include <tf2_ros/buffer.h>
#include <tf2_ros/transform_broadcaster.h>
#include <tf2_ros/transform_listener.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>

// MRPT
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/maps/COccupancyGridMap2D.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>

#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/system/COutputLogger.h>
#include <mrpt/graphslam/apps_related/CGraphSlamHandler.h>

#include "mrpt_graphslam_2d/CGraphSlamEngine_ROS.h"
#include "mrpt_graphslam_2d/CGraphSlamEngine_MR.h"
#include "mrpt_graphslam_2d/TUserOptionsChecker_ROS.h"

// cpp headers
#include <string>
#include <sstream>
#include <vector>

namespace mrpt
{
namespace graphslam
{
namespace apps
{
/**\brief Manage variables, ROS parameters and everything else related to the
 * graphslam-engine ROS wrapper.
 */
template <class GRAPH_T = mrpt::graphs::CNetworkOfPoses2DInf>
class CGraphSlamHandler_ROS : public CGraphSlamHandler<GRAPH_T>
{
   public:
	/**\brief type of graph constraints */
	typedef typename GRAPH_T::constraint_t constraint_t;
	/**\brief type of underlying poses (2D/3D). */
	typedef typename GRAPH_T::constraint_t::type_value pose_t;

	/**\brief Handy self type */
	typedef CGraphSlamHandler_ROS<GRAPH_T> self_t;
	/**\brief Handy parent type */
	typedef CGraphSlamHandler<GRAPH_T> parent_t;

	CGraphSlamHandler_ROS(
		mrpt::system::COutputLogger* logger,
		TUserOptionsChecker<GRAPH_T>* options_checker, ros::NodeHandle* nh_in);
	~CGraphSlamHandler_ROS();

	void getParamsAsString(std::string* str_out);
	std::string getParamsAsString();

	/**\brief Read the problem configuration parameters
	 *
	 * \sa readROSParameters, printParams
	 */
	void readParams();
	/**\brief Print in a compact manner the overall problem configuration
	 * parameters
	 */
	void printParams();

	/**\name Sniffing methods
	 *
	 * Sniff measurements off their corresponding topics
	 */
	/**\{*/
	/**\brief Callback method for handling incoming odometry measurements in a
	 * ROS topic.
	 */
	void sniffOdom(const nav_msgs::Odometry::ConstPtr& ros_odom);
	/**\brief Callback method for handling incoming LaserScans objects in a ROS
	 * topic.
	 */
	void sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan);
	void sniffCameraImage();
	/** TODO - Implement this */
	void sniff3DPointCloud();
	/**\}*/
	/**\brief Indicate whether graphslam execution can proceed normally.
	 * \return False if user has demanded to exit (pressed <C-c>), True
	 * otherwise
	 */
	bool continueExec();
	/**\brief Generate the relevant report directory/files after the graphSLAM
	 * execution.
	 */
	void generateReport();
	/**\brief Provide feedback about the SLAM operation using ROS publilshers,
	 * update the registered frames using the tf2_ros::TransformBroadcaster
	 *
	 * Method makes the necessary calls to all the publishers of the class and
	 * broadcaster instances
	 *
	 * \sa continueExec
	 * \return True if the graphSLAM execution is to continue normally, false
	 * otherwise
	 */
	bool usePublishersBroadcasters();
	/**\brief Wrapper method around the protected setup* class methods.
	 *
	 * Handy for setting up publishers, subscribers, services, TF-related stuff
	 * all at once from the user application
	 *
	 * \note method should be called right after the call to
	 * CGraphSlamHandler_ROS::readParams method
	 */
	void setupComm();

	/**\brief Initialize the CGraphslamEngine_* object
	 *
	 * The CGraphSlamEngine instance is to be instaniated depending on the user
	 * application at hand. User should call this method just after reading the
	 * problem parameters.
	 */
	/**\{*/
	void initEngine_ROS();
	void initEngine_MR();
	/**\}*/

	static const std::string sep_header;
	static const std::string sep_subheader;

   private:
	/**\brief Process an incoming measurement.
	 *
	 * Method is a wrapper around the _process method
	 *
	 * \note Method is automatically called when a new measurement is fetched
	 * from a subscribed topic
	 *
	 * \sa _process
	 */
	void processObservation(mrpt::obs::CObservation::Ptr& observ);
	/**\brief Low level wrapper for executing the
	 * CGraphSlamEngine_ROS::execGraphSlamStep method
	 *
	 * \sa processObservation();
	 */
	void _process(mrpt::obs::CObservation::Ptr& observ);
	/**\brief read configuration parameters from the ROS parameter server.
	 *
	 * \sa readParams
	 */
	void readROSParameters();
	void readStaticTFs();
	/**\brief Fill in the given string with the parameters that have been read
	 * from the ROS parameter server
	 *
	 * \sa getParamsAsString, readROSParameters
	 */
	void getROSParameters(std::string* str_out);
	/**\brief Verify that the parameters read are valid and can be used with the
	 * CGraphSlamEngine_ROS instance.
	 */
	void verifyUserInput();

	/**\brief Reset the flags indicating whether we have received new data
	 * (odometry, laser scans etc.)
	 */
	void resetReceivedFlags();
	/**\name setup* ROS-related methods
	 *\brief Methods for setting up topic subscribers, publishers, and
	 * corresponding services
	 *
	 * \sa setupComm
	 */
	/**\{*/
	void setupSubs();
	void setupPubs();
	void setupSrvs();
	/**\}*/
	/**\brief Pointer to the Ros NodeHanle instance */
	ros::NodeHandle* m_nh;

	// ROS server parameters
	/**\name node, edge, optimizer modules in string representation */
	/**\{*/
	std::string m_node_reg;
	std::string m_edge_reg;
	std::string m_optimizer;
	/**\}*/

	/**\brief Minimum logging level for the current class instance.
	 *
	 * This doesn't affect the logging level of CGraphSlamEngine or any of the
	 * deciders/optimizers.
	 * \note Value is fetched from the ROS Parameter Server (not from the
	 * external .ini file.
	 */
	VerbosityLevel m_min_logging_level;

	/**\name Received measurements - boolean flags
	 *
	 * \brief Flags that indicate if any new measurements have arrived in the
	 * corresponding topics.
	 */
	/**\{*/
	bool m_received_odom;
	bool m_received_laser_scan;
	bool m_received_camera;
	bool m_received_point_cloud;
	/**\}*/

	/**\name Processed measurements
	 *
	 * Measurements that the class can the class instance is keeping track
	 * of and passes to the CGraphSlamEngine_ROS instance.
	 */
	/**\{*/
	/**\brief Received laser scan - converted into MRPT CObservation* format */
	mrpt::obs::CObservationOdometry::Ptr m_mrpt_odom;
	mrpt::obs::CObservation2DRangeScan::Ptr m_mrpt_laser_scan;
	/**\}*/

	/**\name Subscribers - Publishers
	 *
	 * ROS Topic Subscriber/Publisher instances
	 * */
	/**\{*/
	ros::Subscriber m_odom_sub;
	ros::Subscriber m_laser_scan_sub;
	ros::Subscriber m_camera_scan_sub;
	ros::Subscriber m_point_cloud_scan_sub;

	ros::Publisher m_curr_robot_pos_pub;
	ros::Publisher m_robot_trajectory_pub;
	ros::Publisher m_robot_tr_poses_pub;
	ros::Publisher m_gt_trajectory_pub;	 // TODO
	ros::Publisher m_SLAM_eval_metric_pub;	// TODO
	ros::Publisher m_odom_trajectory_pub;
	ros::Publisher m_gridmap_pub;
	ros::Publisher m_stats_pub;
	/**\}*/

	/**\name Topic Names
	 *
	 * Names of the topics that the class instance subscribes or publishes to
	 */
	/**\{*/
	std::string m_odom_topic;
	std::string m_laser_scan_topic;
	std::string m_camera_topic;
	std::string m_point_cloud_topic;

	std::string m_curr_robot_pos_topic;
	std::string m_robot_trajectory_topic;
	std::string m_robot_tr_poses_topic;
	std::string m_odom_trajectory_topic;
	std::string m_gridmap_topic;
	std::string m_stats_topic;
	/**\}*/

	/**\name TransformBroadcasters - TransformListeners
	 */
	/**\{*/
	tf2_ros::Buffer m_buffer;
	tf2_ros::TransformBroadcaster m_broadcaster;
	/**\}*/

	/**\name TF Frame IDs
	 * Names of the current TF Frames available
	 */
	/**\{*/
	/**\brief Frame that the robot starts from. In a single-robot SLAM
	 * setup this can be set to the world frame
	 */
	std::string m_anchor_frame_id;
	std::string m_base_link_frame_id;
	std::string m_odom_frame_id;
	/**\}*/

	/**\name Geometrical Configuration
	 * \brief Variables that setup the geometrical dimensions, distances between
	 * the different robot parts etc.
	 */
	/**\{*/
	geometry_msgs::TransformStamped m_anchor_odom_transform;
	/**\}*/

	/**\brief Odometry path of the robot.
	 * Handy mostly for visualization reasons.
	 */
	nav_msgs::Path m_odom_path;

	/**\brief Times a messge has been published => usePublishersBroadcasters
	 * method is called
	 */
	int m_pub_seq;
	int m_stats_pub_seq;

	/**\brief Total counter of the processed measurements
	 */
	size_t m_measurement_cnt;

	/**\brief ROS topic publisher standard queue size */
	int m_queue_size;

	size_t m_graph_nodes_last_size;

	/**\brief Initial offset of the received odometry.
	 *
	 * Assumption is that in the beginning I have 0 position, thus the incoming
	 * odometry for the algorithm has to be 0 */
	bool m_first_time_in_sniff_odom;
	pose_t m_input_odometry_offset;
};

}  // namespace apps
}  // namespace graphslam
}  // namespace mrpt

#include "mrpt_graphslam_2d/CGraphSlamHandler_ROS_impl.h"
