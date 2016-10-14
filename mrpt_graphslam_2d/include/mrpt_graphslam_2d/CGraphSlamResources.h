/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2016, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#ifndef CGRAPHSLAMRESOURCES_H
#define CGRAPHSLAMRESOURCES_H

// ROS
#include <ros/ros.h>
#include <mrpt_bridge/mrpt_bridge.h>
#include <mrpt_msgs/Pose2DStamped.h>
#include <sensor_msgs/LaserScan.h>

// MRPT
#include <mrpt/graphs/CNetworkOfPoses.h>
#include <mrpt/obs/CObservationOdometry.h>
#include <mrpt/obs/CObservation2DRangeScan.h>
#include <mrpt/system/threads.h>
#include <mrpt/system/string_utils.h>
#include <mrpt/system/filesystem.h>
#include <mrpt/system/os.h>
#include <mrpt/utils/mrpt_macros.h>
#include <mrpt/utils/COutputLogger.h>
#include <mrpt/graphslam/CGraphSlamEngine.h>
#include <mrpt/graphslam/apps_related/TUserOptionsChecker.h>
#include <mrpt/graphslam/apps_related/CGraphSlamHandler.h>

// cpp headers
#include <string>
#include <sstream>
#include <vector>

/**\brief Manage variables, ROS parameters and everything else related to the
 * graphslam-engine ROS wrapper.
 */
class CGraphSlamResources
{
public:
	CGraphSlamResources(
			mrpt::utils::COutputLogger* logger_in,
			ros::NodeHandle* nh
			);
	~CGraphSlamResources();


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
	/**\brief Callback method for handling incoming odometry measurements in a ROS
	 * topic.
	 */
	void sniffOdom(const mrpt_msgs::Pose2DStamped::ConstPtr& ros_odom);
	/**\brief Callback method for handling incoming LaserScans objects in a ROS
	 * topic.
	 */
	void sniffLaserScan(const sensor_msgs::LaserScan::ConstPtr& ros_laser_scan);
	void sniffCameraImage();
	/** TODO - Implement this */
	void sniff3DPointCloud();
	/**\}*/
	/**\brief Indicate whether graphslam execution can proceed normally.
	 * \return False if user has demanded to exit (pressed <C-c>), True otherwise
	 */
	bool continueExec();
	/**\brief Generate the relevant report directory/files after the graphSLAM
	 * execution.
	 */
	void generateReport();

	
	static const std::string sep_header;
	static const std::string sep_subheader;
private:
	/**\brief Initialize the CGraphslamEngine object based on the user input. */
	void initGraphSLAM();
	/**\brief Process an incoming measurement.
	 *
	 * Method is a wrapper around the _process method 
	 *
	 * \note Method is automatically called when a new measurement is fetched
	 * from a subscribed topic
	 *
	 * \sa _process
	 */
	void processObservation(mrpt::obs::CObservationPtr& observ);
	/**\brief Low level wrapper for executing the
	 * CGraphSlamEngine::execGraphSlamStep method
	 *
	 * \sa processObservation();
	 */
	void _process(mrpt::obs::CObservationPtr& observ);
	/**\brief read configuration parameters from the ROS parameter server.
	 *
	 * \sa readParams
	 */
	void readROSParameters();
	/**\brief Fill in the given string with the parameters that have been read
	 * from the ROS parameter server
	 *
	 * \sa getParamsAsString, readROSParameters
	 */
	void getROSParameters(std::string* str_out);
	/**\brief Verify that the parameters read are valid and can be used with the
	 * CGraphSlamEngine instance.
	 */
	void verifyUserInput();

	void resetReceivedFlags();
	/**\name Methods for settign up TOPIC subscribers, publishers, and
	 * corresponding services
	 */
	/**\{*/
	void setupSubs();
	void setupPubs();
	void setupSrvs();
	/**\}*/

	/**\brief Pointer to the logging instance */
	mrpt::utils::COutputLogger* m_logger;
	ros::NodeHandle* nh;

	// ROS server parameters
	/**\name node, edge, optimizer modules in string representation */
	/**\{*/
	std::string m_node_reg;
	std::string m_edge_reg;
	std::string m_optimizer;
	/**\}*/

	/**\name graphslam-engine various filenames */
	/**\{*/
	std::string m_ini_fname; /**<.ini configuration file */
	std::string m_gt_fname; /**<ground-truth filename */
	/**\}*/

	/**\brief Minimum logging level
	 *
	 * \note Value is fetched  from the ROS Parameter Server (not from the
	 * external .ini file.
	 */
	mrpt::utils::VerbosityLevel m_min_logging_level;

	/**\brief Are visuals on? */
	bool m_disable_visuals;

	/**\brief Struct instance holding the available deciders/optimizers that the
	 * user can issue
	 */
	mrpt::graphslam::supplementary::TUserOptionsChecker m_graphslam_opts;
	CGraphSlamHandler* m_graphslam_handler;

	bool m_has_read_config;

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
	
	/**\}*/
	/**\name Processed measurements
	 *
	 * Measurements that the class can the class instance is keeping track
	 * of and passes to the CGraphSlamEngine instance.
	 */
	/**\{*/
	/**\brief Received laser scan - converted into MRPT CObservation* format */
	mrpt::obs::CObservationOdometryPtr m_mrpt_odom;
	mrpt::obs::CObservation2DRangeScanPtr m_mrpt_laser_scan;
	/**\}*/

	mrpt::graphslam::CGraphSlamEngine<mrpt::graphs::CNetworkOfPoses2DInf>*
		m_graphslam_engine;

	/**\name Subscribers
	 *
	 * ROS Topic Subscriber instances
	 * */
	/**\{*/
	ros::Subscriber m_odom_sub;
	ros::Subscriber m_laser_scan_sub;
	ros::Subscriber m_camera_scan_sub;
	ros::Subscriber m_point_cloud_scan_sub;
	/**\}*/

	/**\name Topic names
	 *
	 * Names of the topics that the class instance subscribes or publishes to
	 */
	/**\{*/
	std::string m_odom_topic;
	std::string m_laser_scan_topic;
	std::string m_camera_topic;
	std::string m_point_cloud_topic;
	/**\}*/

	/**\brief Total counter of the processed measurements
	 */
	size_t m_measurement_cnt;
};

#endif /* end of include guard: CGRAPHSLAMRESOURCES_H */
